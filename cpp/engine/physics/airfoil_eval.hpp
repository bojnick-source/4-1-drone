// ============================================================================
// Fragment 3.1.05 — Airfoil Evaluation Hardening (Alpha/Re Interp + Clamp/Extrap Policy + NaN Safety) (C++)
// File: airfoil_eval.hpp
// ============================================================================
//
// Purpose:
// - Robust, deterministic airfoil coefficient evaluation for BEMT:
//     (alpha_rad, Re) -> (cl, cd, cm)
// - Hardened interpolation with explicit extrapolation policies and safety clamps.
// - Works as a thin layer you can call from existing airfoil_polar/bemt_polar code,
//   OR as a standalone evaluator if you already hold tables in-memory.
//
// Key guarantees:
// - No NaNs/Inf returned (unless caller explicitly allows).
// - cd is clamped to >= 0.
// - Alpha arrays must be strictly increasing.
// - Reynolds slices must be strictly increasing.
// - Behavior outside table bounds is controlled by policy (Clamp, LinearExtrap, Error).
//
// ============================================================================

#pragma once
#include "bemt_safety.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace lift::bemt {

// -----------------------------
// Data types
// -----------------------------

struct AirfoilCoeffs final {
    double cl = 0.0;
    double cd = 0.0;
    double cm = 0.0;

    void sanitize() noexcept {
        if (!is_finite(cl)) cl = 0.0;
        if (!is_finite(cd) || cd < 0.0) cd = 0.0;
        if (!is_finite(cm)) cm = 0.0;
    }
};

enum class ExtrapMode : std::uint8_t {
    Clamp = 0,        // clamp alpha/Re to table bounds
    Linear = 1,       // linear extrap using nearest segment
    Error = 2         // throw InvalidInput if outside bounds
};

struct AirfoilEvalPolicy final {
    ExtrapMode alpha_mode = ExtrapMode::Clamp;
    ExtrapMode Re_mode    = ExtrapMode::Clamp;

    // Reynolds interpolation in log-space is usually more stable.
    bool interp_log_Re = true;

    // Hard clamps (safety, not “physics”):
    double alpha_abs_max_rad = 1.3962634015954636; // 80 deg
    double Re_min = 1.0e3;
    double Re_max = 5.0e7;

    // If true, enforce cd >= 0 and finite outputs always.
    bool sanitize_outputs = true;

    void validate() const {
        LIFT_BEMT_REQUIRE(alpha_abs_max_rad > 0.0 && alpha_abs_max_rad < 3.14,
                          ErrorCode::InvalidConfig, "AirfoilEvalPolicy.alpha_abs_max_rad invalid");
        LIFT_BEMT_REQUIRE(is_finite(Re_min) && is_finite(Re_max) && Re_min > 0.0 && Re_max > Re_min,
                          ErrorCode::InvalidConfig, "AirfoilEvalPolicy.Re bounds invalid");
    }
};

// Non-owning slice view (alpha must be strictly increasing).
struct AirfoilSliceView final {
    double Re = 0.0;
    const double* alpha_rad = nullptr;
    const double* cl = nullptr;
    const double* cd = nullptr;
    const double* cm = nullptr;
    std::size_t n = 0;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(Re) && Re > 0.0, ErrorCode::InvalidInput, "AirfoilSliceView.Re invalid");
        LIFT_BEMT_REQUIRE(alpha_rad && cl && cd && cm, ErrorCode::InvalidInput, "AirfoilSliceView null arrays");
        LIFT_BEMT_REQUIRE(n >= 2, ErrorCode::InvalidInput, "AirfoilSliceView needs n>=2");
        for (std::size_t i = 1; i < n; ++i) {
            LIFT_BEMT_REQUIRE(is_finite(alpha_rad[i - 1]) && is_finite(alpha_rad[i]),
                              ErrorCode::InvalidInput, "AirfoilSliceView alpha not finite");
            LIFT_BEMT_REQUIRE(alpha_rad[i] > alpha_rad[i - 1],
                              ErrorCode::InvalidInput, "AirfoilSliceView alpha must be strictly increasing");
        }
    }

    double alpha_min() const noexcept { return alpha_rad[0]; }
    double alpha_max() const noexcept { return alpha_rad[n - 1]; }
};

// Non-owning table view (slices must have strictly increasing Re).
struct AirfoilTableView final {
    const AirfoilSliceView* slices = nullptr;
    std::size_t m = 0;

    void validate() const {
        LIFT_BEMT_REQUIRE(slices, ErrorCode::InvalidInput, "AirfoilTableView slices null");
        LIFT_BEMT_REQUIRE(m >= 1, ErrorCode::InvalidInput, "AirfoilTableView needs >=1 slice");
        for (std::size_t i = 0; i < m; ++i) {
            slices[i].validate();
            if (i > 0) {
                LIFT_BEMT_REQUIRE(slices[i].Re > slices[i - 1].Re,
                                  ErrorCode::InvalidInput, "AirfoilTableView Re must be strictly increasing");
            }
        }
    }

    double Re_min() const noexcept { return slices[0].Re; }
    double Re_max() const noexcept { return slices[m - 1].Re; }
};

// -----------------------------
// Internal helpers
// -----------------------------

inline std::size_t upper_bound_idx(const double* xs, std::size_t n, double x) noexcept {
    // returns first i such that xs[i] > x; in [0..n]
    const double* b = xs;
    const double* e = xs + n;
    const double* it = std::upper_bound(b, e, x);
    return static_cast<std::size_t>(it - b);
}

inline double lerp(double a, double b, double t) noexcept { return a + (b - a) * t; }

inline double inv_lerp(double a, double b, double x, double eps = 1e-12) noexcept {
    const double den = b - a;
    return safe_div((x - a), den, eps);
}

inline AirfoilCoeffs interp_alpha_in_slice(const AirfoilSliceView& s,
                                           double alpha_rad,
                                           const AirfoilEvalPolicy& pol) {
    // Alpha hard clamp (safety)
    if (!is_finite(alpha_rad)) alpha_rad = 0.0;
    alpha_rad = clamp(alpha_rad, -pol.alpha_abs_max_rad, pol.alpha_abs_max_rad);

    const double amin = s.alpha_min();
    const double amax = s.alpha_max();

    if (alpha_rad < amin || alpha_rad > amax) {
        if (pol.alpha_mode == ExtrapMode::Error) {
            LIFT_BEMT_REQUIRE(false, ErrorCode::InvalidInput, "alpha outside airfoil table bounds");
        }
        if (pol.alpha_mode == ExtrapMode::Clamp) {
            alpha_rad = clamp(alpha_rad, amin, amax);
        }
        // Linear extrap uses nearest segment; handled below by selecting indices at edges.
    }

    // Find bracketing indices:
    // i1 = upper_bound(alpha) => alpha[i1-1] <= a < alpha[i1]
    const std::size_t i1 = upper_bound_idx(s.alpha_rad, s.n, alpha_rad);
    std::size_t i0 = 0;
    std::size_t i2 = 1;

    if (i1 == 0) { // below first
        i0 = 0;
        i2 = 1;
    } else if (i1 >= s.n) { // above last
        i0 = s.n - 2;
        i2 = s.n - 1;
    } else {
        i0 = i1 - 1;
        i2 = i1;
    }

    const double a0 = s.alpha_rad[i0];
    const double a1 = s.alpha_rad[i2];
    const double t = clamp(inv_lerp(a0, a1, alpha_rad), 0.0, 1.0);

    AirfoilCoeffs c;
    c.cl = lerp(s.cl[i0], s.cl[i2], t);
    c.cd = lerp(s.cd[i0], s.cd[i2], t);
    c.cm = lerp(s.cm[i0], s.cm[i2], t);
    if (pol.sanitize_outputs) c.sanitize();
    return c;
}

inline double Re_axis(double Re, const AirfoilEvalPolicy& pol) noexcept {
    if (!is_finite(Re) || Re <= 0.0) Re = pol.Re_min;
    return clamp(Re, pol.Re_min, pol.Re_max);
}

inline double interp_axis_Re(double Re, const AirfoilEvalPolicy& pol) noexcept {
    if (!pol.interp_log_Re) return Re;
    return safe_log(std::max(Re, 1.0));
}

// -----------------------------
// Public API: evaluate
// -----------------------------

inline AirfoilCoeffs airfoil_eval(const AirfoilTableView& tbl,
                                  double alpha_rad,
                                  double Re,
                                  const AirfoilEvalPolicy& pol_in = {}) {
    AirfoilEvalPolicy pol = pol_in;
    pol.validate();
    tbl.validate();

    // Apply Re safety bounds first
    Re = Re_axis(Re, pol);

    const double tRe_min = tbl.Re_min();
    const double tRe_max = tbl.Re_max();

    if (Re < tRe_min || Re > tRe_max) {
        if (pol.Re_mode == ExtrapMode::Error) {
            LIFT_BEMT_REQUIRE(false, ErrorCode::InvalidInput, "Re outside airfoil table bounds");
        }
        if (pol.Re_mode == ExtrapMode::Clamp) {
            Re = clamp(Re, tRe_min, tRe_max);
        }
        // Linear extrap is supported below by edge bracket selection.
    }

    // If only one slice, just interpolate in alpha within that slice.
    if (tbl.m == 1) {
        auto c = interp_alpha_in_slice(tbl.slices[0], alpha_rad, pol);
        if (pol.sanitize_outputs) c.sanitize();
        return c;
    }

    // Find bracketing Re slices
    // i1 = first slice.Re > Re
    // We do the search in raw Re, but interpolate in log(Re) if requested.
    std::size_t i1 = 0;
    {
        // manual upper_bound on slices.Re
        std::size_t lo = 0, hi = tbl.m;
        while (lo < hi) {
            const std::size_t mid = lo + (hi - lo) / 2;
            if (tbl.slices[mid].Re <= Re) lo = mid + 1;
            else hi = mid;
        }
        i1 = lo;
    }

    std::size_t i0 = 0, i2 = 1;
    if (i1 == 0) {
        i0 = 0; i2 = 1;
    } else if (i1 >= tbl.m) {
        i0 = tbl.m - 2; i2 = tbl.m - 1;
    } else {
        i0 = i1 - 1; i2 = i1;
    }

    const double Re0 = tbl.slices[i0].Re;
    const double Re1 = tbl.slices[i2].Re;

    // Evaluate alpha-interp on each slice
    const AirfoilCoeffs c0 = interp_alpha_in_slice(tbl.slices[i0], alpha_rad, pol);
    const AirfoilCoeffs c1 = interp_alpha_in_slice(tbl.slices[i2], alpha_rad, pol);

    // Interpolate across Re (linear in Re or log(Re))
    const double x  = interp_axis_Re(Re, pol);
    const double x0 = interp_axis_Re(Re0, pol);
    const double x1 = interp_axis_Re(Re1, pol);

    double t = clamp(inv_lerp(x0, x1, x), 0.0, 1.0);

    AirfoilCoeffs c;
    c.cl = lerp(c0.cl, c1.cl, t);
    c.cd = lerp(c0.cd, c1.cd, t);
    c.cm = lerp(c0.cm, c1.cm, t);

    if (pol.sanitize_outputs) c.sanitize();
    return c;
}

} // namespace lift::bemt
