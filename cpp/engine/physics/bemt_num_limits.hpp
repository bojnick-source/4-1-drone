// ============================================================================
// Fragment 3.1.09 — BEMT Numerical Limits (Global Clamps + Deterministic Fail/Fallback Policy) (C++)
// File: bemt_num_limits.hpp
// ============================================================================
//
// Purpose:
// - Centralize solver-wide numeric clamps and fallback policies to prevent drift,
//   inconsistent “magic numbers,” and NaN propagation.
// - Used by hover/forward solvers, airfoil eval, loss models, and closeout.
//
// Policy goals:
// - Deterministic behavior for extreme/degenerate inputs.
// - Explicit “what gets clamped” vs “what triggers failure.”
// - Keep optimization loop stable: prefer clamp + error codes over exceptions
//   inside inner loops; reserve throws for configuration/contract violations.
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"
#include "bemt_safety.hpp"

#include <cstdint>
#include <limits>
#include <string>

namespace lift::bemt {

enum class ClampPolicy : std::uint8_t {
    ClampAndContinue = 0,   // clamp values into safe range
    FailSoft = 1,           // return NotConverged/NumericalFailure (no throw)
    ThrowHard = 2           // throw BemtError (configuration/contract breach)
};

struct BemtNumLimits final {
    // --- Global finite guards ---
    double eps = 1e-12;              // generic epsilon for division guards
    double min_positive = 1e-24;      // for sqrt/log denominators

    // --- Angle clamps (radians) ---
    double phi_abs_max = 1.5533430342749532; // 89.0 deg (avoid sin/cos singularities)
    double alpha_abs_max = 1.3962634015954636; // 80 deg

    // --- Reynolds bounds (sanity) ---
    double Re_min = 1e3;
    double Re_max = 5e7;

    // --- Induced / nondimensional bounds ---
    // Used in forward inflow approximations, guard rails for optimizer
    double lambda_min = -5.0;
    double lambda_max = 5.0;
    double mu_min = 0.0;
    double mu_max = 5.0;

    // --- Power/torque clamps (do not enforce physics; just stop catastrophes) ---
    double power_max_W = 1e9;         // hard ceiling to stop blow-ups
    double torque_max_Nm = 1e7;
    double thrust_max_N = 1e9;

    // --- Loss factor floors (avoid divide-by-zero in momentum terms) ---
    double F_min = 1e-3;              // allow near-zero but not 0 when used as divisor
    double F_max = 1.0;

    // --- Station integration ---
    double dr_min_m = 0.0;            // can be raised to avoid near-zero annuli
    double dr_max_m = 1e9;

    // --- Behavior ---
    ClampPolicy policy = ClampPolicy::ClampAndContinue;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(eps) && eps > 0.0 && eps < 1e-6, ErrorCode::InvalidConfig, "BemtNumLimits.eps invalid");
        LIFT_BEMT_REQUIRE(is_finite(min_positive) && min_positive > 0.0 && min_positive < 1e-9, ErrorCode::InvalidConfig, "BemtNumLimits.min_positive invalid");

        LIFT_BEMT_REQUIRE(is_finite(phi_abs_max) && phi_abs_max > 0.5 && phi_abs_max <= 1.5707963267948966, ErrorCode::InvalidConfig, "phi_abs_max invalid");
        LIFT_BEMT_REQUIRE(is_finite(alpha_abs_max) && alpha_abs_max > 0.5 && alpha_abs_max < 3.14, ErrorCode::InvalidConfig, "alpha_abs_max invalid");

        LIFT_BEMT_REQUIRE(is_finite(Re_min) && is_finite(Re_max) && Re_min > 0.0 && Re_max > Re_min, ErrorCode::InvalidConfig, "Re bounds invalid");

        LIFT_BEMT_REQUIRE(is_finite(lambda_min) && is_finite(lambda_max) && lambda_max > lambda_min, ErrorCode::InvalidConfig, "lambda bounds invalid");
        LIFT_BEMT_REQUIRE(is_finite(mu_min) && is_finite(mu_max) && mu_max >= mu_min && mu_min >= 0.0, ErrorCode::InvalidConfig, "mu bounds invalid");

        LIFT_BEMT_REQUIRE(is_finite(power_max_W) && power_max_W > 0.0, ErrorCode::InvalidConfig, "power_max_W invalid");
        LIFT_BEMT_REQUIRE(is_finite(torque_max_Nm) && torque_max_Nm > 0.0, ErrorCode::InvalidConfig, "torque_max_Nm invalid");
        LIFT_BEMT_REQUIRE(is_finite(thrust_max_N) && thrust_max_N > 0.0, ErrorCode::InvalidConfig, "thrust_max_N invalid");

        LIFT_BEMT_REQUIRE(is_finite(F_min) && is_finite(F_max) && F_min > 0.0 && F_max >= F_min && F_max <= 1.0, ErrorCode::InvalidConfig, "F bounds invalid");

        LIFT_BEMT_REQUIRE(is_finite(dr_min_m) && is_finite(dr_max_m) && dr_max_m > dr_min_m && dr_min_m >= 0.0, ErrorCode::InvalidConfig, "dr bounds invalid");
    }
};

// Utility: clamp with policy-aware response.
inline double clamp_or_flag(double x, double lo, double hi, bool& clamped) noexcept {
    clamped = false;
    if (!is_finite(x)) { clamped = true; return clamp(0.0, lo, hi); }
    if (x < lo) { clamped = true; return lo; }
    if (x > hi) { clamped = true; return hi; }
    return x;
}

struct ClampDecision final {
    ErrorCode code = ErrorCode::Ok;
    std::string message;
    bool any_clamped = false;
};

// Apply a clamp under a policy; if FailSoft/ThrowHard, you decide upstream how to act.
inline ClampDecision apply_policy(const BemtNumLimits& lim, bool any_clamped, const char* what) {
    ClampDecision d;
    d.any_clamped = any_clamped;
    if (!any_clamped) return d;

    if (lim.policy == ClampPolicy::ClampAndContinue) {
        d.code = ErrorCode::Ok;
        d.message = std::string("clamped: ") + (what ? what : "");
        return d;
    }
    if (lim.policy == ClampPolicy::FailSoft) {
        d.code = ErrorCode::NonConverged;
        d.message = std::string("clamp triggered FailSoft: ") + (what ? what : "");
        return d;
    }
    // ThrowHard
    LIFT_BEMT_REQUIRE(false, ErrorCode::InvalidConfig, what ? what : "Clamp triggered ThrowHard");
    return d; // unreachable; placates compiler
}

// Common clamps
inline double clamp_phi(double phi_rad, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(phi_rad, -lim.phi_abs_max, lim.phi_abs_max, clamped);
}

inline double clamp_alpha(double alpha_rad, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(alpha_rad, -lim.alpha_abs_max, lim.alpha_abs_max, clamped);
}

inline double clamp_Re(double Re, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(Re, lim.Re_min, lim.Re_max, clamped);
}

inline double clamp_F(double F, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(F, lim.F_min, lim.F_max, clamped);
}

inline double clamp_power(double P_W, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(P_W, 0.0, lim.power_max_W, clamped);
}

inline double clamp_torque(double Q_Nm, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(Q_Nm, -lim.torque_max_Nm, lim.torque_max_Nm, clamped);
}

inline double clamp_thrust(double T_N, const BemtNumLimits& lim, bool& clamped) noexcept {
    return clamp_or_flag(T_N, 0.0, lim.thrust_max_N, clamped);
}

} // namespace lift::bemt
