// ============================================================================
// Fragment 3.1.02 — BEMT Safety Utilities (Finite/Clamp/Safe Math + Iteration Guardrails) (C++)
// File: bemt_safety.hpp
// ============================================================================
//
// Goal:
// - Centralize “hardening” utilities used by BEMT hover/forward solvers.
// - Provide numerically safe primitives, clamp policies, and robust convergence tracking.
// - No solver logic here; only helpers that reduce NaNs/infs and improve determinism.
//
// Design notes:
// - Non-allocating, header-only, constexpr-friendly where sensible.
// - Uses bemt_require.hpp for consistent ErrorCode + exceptions.
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"
#include "bemt_error.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace lift::bemt {

// -----------------------------
// Finite helpers
// -----------------------------
inline double clamp01(double x) noexcept { return clamp(x, 0.0, 1.0); }

// -----------------------------
// Safe math
// -----------------------------

inline double safe_sqrt(double x, double eps = 0.0) noexcept {
    // If eps>0, clamp x to >=eps to avoid sqrt(0) downstream divisions.
    const double y = (x < eps) ? eps : x;
    return std::sqrt(y);
}

inline double safe_log(double x, double min_x = 1e-300) noexcept {
    return std::log((x < min_x) ? min_x : x);
}

inline double safe_exp(double x, double max_x = 700.0) noexcept {
    // exp(709) ~ 8e307 near DBL_MAX
    return std::exp((x > max_x) ? max_x : x);
}

inline double safe_acos(double x) noexcept { return std::acos(clamp(x, -1.0, 1.0)); }
inline double safe_asin(double x) noexcept { return std::asin(clamp(x, -1.0, 1.0)); }

inline double safe_atan2(double y, double x) noexcept {
    // atan2 handles infinities, but we still keep behavior well-defined if both 0.
    if (y == 0.0 && x == 0.0) return 0.0;
    return std::atan2(y, x);
}

// Wrap angle to (-pi, pi]
inline double wrap_pi(double a) noexcept {
    constexpr double two_pi = 2.0 * 3.141592653589793238462643383279502884;
    if (!is_finite(a)) return 0.0;
    a = std::fmod(a, two_pi);
    if (a <= -3.141592653589793238462643383279502884) a += two_pi;
    if (a >   3.141592653589793238462643383279502884) a -= two_pi;
    return a;
}

// -----------------------------
// Robust convergence tracking
// -----------------------------

enum class IterStopReason : std::uint8_t {
    Converged = 0,
    MaxIter,
    NumericalFailure,
    Diverged,
    InvalidInput
};

struct IterationStats final {
    int iters = 0;
    double last_abs_err = std::numeric_limits<double>::infinity();
    double last_rel_err = std::numeric_limits<double>::infinity();
    IterStopReason reason = IterStopReason::MaxIter;

    bool converged() const noexcept { return reason == IterStopReason::Converged; }
};

struct IterationConfig final {
    int max_iter = 200;
    double tol_abs = 1e-10;
    double tol_rel = 1e-6;

    // Damping limits for fixed-point style updates:
    // x_new = (1-damp)*x_old + damp*x_candidate
    double damp_min = 0.05;
    double damp_max = 1.0;

    // Divergence detection
    double rel_err_growth_limit = 50.0; // if rel_err explodes by this factor, consider diverged
    int growth_window = 6;

    void validate() const {
        LIFT_BEMT_REQUIRE(max_iter >= 10 && max_iter <= 20000, ErrorCode::InvalidConfig, "IterationConfig.max_iter invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_abs) && tol_abs > 0.0 && tol_abs < 1e-3, ErrorCode::InvalidConfig, "IterationConfig.tol_abs invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_rel) && tol_rel > 0.0 && tol_rel < 1e-2, ErrorCode::InvalidConfig, "IterationConfig.tol_rel invalid");
        LIFT_BEMT_REQUIRE(is_finite(damp_min) && is_finite(damp_max) && damp_min > 0.0 && damp_max >= damp_min && damp_max <= 1.0,
                          ErrorCode::InvalidConfig, "IterationConfig.damp invalid");
        LIFT_BEMT_REQUIRE(is_finite(rel_err_growth_limit) && rel_err_growth_limit >= 5.0 && rel_err_growth_limit <= 1e6,
                          ErrorCode::InvalidConfig, "IterationConfig.rel_err_growth_limit invalid");
        LIFT_BEMT_REQUIRE(growth_window >= 3 && growth_window <= 50, ErrorCode::InvalidConfig, "IterationConfig.growth_window invalid");
    }
};

inline double abs_err(double a, double b) noexcept { return std::fabs(a - b); }

inline double rel_err(double a, double b, double eps = 1e-12) noexcept {
    // Relative to scale of b (or a if b is tiny). Stable for near-zero.
    const double scale = std::max({std::fabs(a), std::fabs(b), eps});
    return std::fabs(a - b) / scale;
}

inline bool is_converged(double abs_e, double rel_e, const IterationConfig& cfg) noexcept {
    return (abs_e <= cfg.tol_abs) || (rel_e <= cfg.tol_rel);
}

struct DivergenceGuard final {
    // Tracks recent rel errors to detect blow-ups.
    int window = 6;
    double growth_limit = 50.0;

    int filled = 0;
    double first = 0.0;
    double last = 0.0;

    void reset(int win, double limit) noexcept {
        window = win;
        growth_limit = limit;
        filled = 0;
        first = 0.0;
        last = 0.0;
    }

    // Returns true if diverged.
    bool update(double rel_e) noexcept {
        if (!is_finite(rel_e)) return true;
        if (filled == 0) first = rel_e;
        last = rel_e;
        filled++;
        if (filled < window) return false;

        // If rel error grew massively over the window, consider diverged.
        const double denom = std::max(first, 1e-12);
        const double ratio = last / denom;
        return (ratio >= growth_limit);
    }
};

inline double damped_update(double x_old, double x_candidate, double damp) noexcept {
    damp = clamp(damp, 0.0, 1.0);
    return (1.0 - damp) * x_old + damp * x_candidate;
}

// -----------------------------
// Range sanity checks
// -----------------------------

inline void require_in_range(double x, double lo, double hi, ErrorCode code, const char* msg) {
    LIFT_BEMT_REQUIRE(is_finite(x) && x >= lo && x <= hi, code, msg);
}

inline void require_nonneg(double x, ErrorCode code, const char* msg) {
    LIFT_BEMT_REQUIRE(is_finite(x) && x >= 0.0, code, msg);
}

} // namespace lift::bemt
