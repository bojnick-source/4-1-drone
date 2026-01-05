// ============================================================================
// Fragment 3.1.08 — BEMT Integrity Checks (Power/Torque Consistency, FM, Disk Loading, Sanity Gates) (C++)
// File: bemt_integrity.hpp
// ============================================================================
//
// Purpose:
// - Provide hardened, reusable “integrity checks” for BEMT outputs.
// - Centralize derived metrics (FM, disk loading, power consistency) with safe math.
// - Produce deterministic pass/fail diagnostics for closeout and GO/NO-GO layers.
//
// This file is solver-agnostic: it works on primitive scalars so you can call it
// from your existing bemt_solver / bemt_forward / mc wrapper.
//
// Definitions:
// - Disk area A = pi * R^2 (unless caller supplies exact geometry area).
// - Disk loading DL = T / A.
// - Ideal induced power (hover) P_ideal = T^(3/2) / sqrt(2*rho*A).
// - Figure of Merit FM = P_ideal / P_actual   (clamped to [0, 1.2] for sanity).
// - Shaft power consistency: P_shaft ≈ Q * omega.
//
// ============================================================================

#pragma once
#include "bemt_safety.hpp"
#include "bemt_require.hpp"
#include "bemt_metrics.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace lift::bemt {

struct IntegrityConfig final {
    // Basic physical checks
    bool require_nonnegative_power = true;
    bool require_nonnegative_thrust = true;

    // Consistency checks
    double torque_power_rel_tol = 0.05;   // |P - Q*omega| / max(P,1) <= tol
    double torque_power_abs_tol = 5.0;    // W, floor

    // FM sanity (solver might compute FM; we can recompute)
    double fm_min = 0.0;
    double fm_max = 1.2;

    // Clamp for derived metrics to prevent NaN propagation
    bool clamp_outputs = true;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(torque_power_rel_tol) && torque_power_rel_tol >= 0.0 && torque_power_rel_tol <= 0.5,
                          ErrorCode::InvalidConfig, "IntegrityConfig.torque_power_rel_tol invalid");
        LIFT_BEMT_REQUIRE(is_finite(torque_power_abs_tol) && torque_power_abs_tol >= 0.0 && torque_power_abs_tol <= 1e6,
                          ErrorCode::InvalidConfig, "IntegrityConfig.torque_power_abs_tol invalid");
        LIFT_BEMT_REQUIRE(is_finite(fm_min) && is_finite(fm_max) && fm_max > fm_min,
                          ErrorCode::InvalidConfig, "IntegrityConfig.fm bounds invalid");
    }
};

struct IntegrityOut final {
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    // Derived metrics
    double disk_area_m2 = 0.0;
    double disk_loading_N_m2 = 0.0;

    double P_shaft_from_Q_W = 0.0; // Q*omega
    double P_ideal_hover_W = 0.0;  // ideal induced hover power
    double FM = 0.0;               // P_ideal / P_actual

    // Consistency measures
    double torque_power_abs_err_W = 0.0;
    double torque_power_rel_err = 0.0;

    bool ok() const noexcept { return code == ErrorCode::Ok; }
};

// Compute disk area from radius (m). Returns 0 if invalid.
inline double disk_area_from_radius(double R_m) noexcept {
    return disk_area(R_m);
}

// Ideal hover induced power (momentum theory), in Watts.
inline double hover_ideal_power(double T_N, double rho_kg_m3, double A_m2) noexcept {
    if (!is_finite(T_N) || !is_finite(rho_kg_m3) || !is_finite(A_m2)) return 0.0;
    if (T_N <= 0.0 || rho_kg_m3 <= 0.0 || A_m2 <= 0.0) return 0.0;
    // P_ideal = T^(3/2)/sqrt(2*rho*A)
    const double num = std::pow(T_N, 1.5);
    const double den = safe_sqrt(2.0 * rho_kg_m3 * A_m2, 1e-24);
    return safe_div(num, den, 1e-24);
}

// Figure of merit (hover). Returns 0 if invalid.
inline double hover_FM(double T_N, double P_W, double rho_kg_m3, double A_m2) noexcept {
    if (!is_finite(P_W) || P_W <= 0.0) return 0.0;
    const double Pid = hover_ideal_power(T_N, rho_kg_m3, A_m2);
    if (Pid <= 0.0) return 0.0;
    return Pid / P_W;
}

// Power from torque and omega (W). Returns 0 if invalid.
inline double power_from_torque(double Q_Nm, double omega_rad_s) noexcept {
    if (!is_finite(Q_Nm) || !is_finite(omega_rad_s)) return 0.0;
    if (omega_rad_s < 0.0) return 0.0;
    const double P = Q_Nm * omega_rad_s;
    return is_finite(P) ? P : 0.0;
}

// Main integrity evaluation.
// Inputs:
// - T_N, Q_Nm, P_W: solver outputs (total rotor)
// - rho: air density
// - disk_area_m2: either exact or computed from radius; must be >0 to compute FM/DL
// - omega_rad_s: needed to check torque/power consistency
inline IntegrityOut bemt_integrity(double T_N,
                                   double Q_Nm,
                                   double P_W,
                                   double rho_kg_m3,
                                   double disk_area_m2,
                                   double omega_rad_s,
                                   const IntegrityConfig& cfg_in = {}) {
    IntegrityConfig cfg = cfg_in;
    cfg.validate();

    IntegrityOut out;

    // Basic finiteness checks
    if (!is_finite(T_N) || !is_finite(Q_Nm) || !is_finite(P_W) || !is_finite(rho_kg_m3) || !is_finite(disk_area_m2) || !is_finite(omega_rad_s)) {
        out.code = ErrorCode::NumericalFailure;
        out.message = "non-finite inputs";
        return out;
    }

    if (cfg.require_nonnegative_thrust && T_N < 0.0) {
        out.code = ErrorCode::InvalidInput;
        out.message = "negative thrust";
        return out;
    }
    if (cfg.require_nonnegative_power && P_W < 0.0) {
        out.code = ErrorCode::InvalidInput;
        out.message = "negative power";
        return out;
    }
    if (rho_kg_m3 <= 0.0) {
        out.code = ErrorCode::InvalidInput;
        out.message = "rho <= 0";
        return out;
    }
    if (disk_area_m2 <= 0.0) {
        // Allow pass-through without FM/DL if caller didn’t provide area.
        out.disk_area_m2 = 0.0;
        out.disk_loading_N_m2 = 0.0;
    } else {
        out.disk_area_m2 = disk_area_m2;
        out.disk_loading_N_m2 = disk_loading(T_N, disk_area_m2);
    }

    // Torque->power consistency
    out.P_shaft_from_Q_W = power_from_torque(Q_Nm, omega_rad_s);

    const double abs_err = std::fabs(P_W - out.P_shaft_from_Q_W);
    const double rel_err = abs_err / std::max(std::fabs(P_W), 1.0);

    out.torque_power_abs_err_W = abs_err;
    out.torque_power_rel_err = rel_err;

    // Derive FM (hover) if area available
    if (disk_area_m2 > 0.0) {
        out.P_ideal_hover_W = hover_ideal_power(T_N, rho_kg_m3, disk_area_m2);
        out.FM = hover_FM(T_N, P_W, rho_kg_m3, disk_area_m2);

        if (cfg.clamp_outputs) {
            out.P_ideal_hover_W = std::max(0.0, out.P_ideal_hover_W);
            out.FM = clamp(out.FM, cfg.fm_min, cfg.fm_max);
            out.disk_loading_N_m2 = std::max(0.0, out.disk_loading_N_m2);
        }

        // FM > 1 is possible only if inputs inconsistent; flag but don’t hard fail by default.
        if (out.FM > cfg.fm_max) {
            out.code = ErrorCode::NumericalFailure;
            out.message = "FM out of bounds (inconsistent power/thrust/area)";
            return out;
        }
    } else {
        out.P_ideal_hover_W = 0.0;
        out.FM = 0.0;
    }

    // Torque-power mismatch gate
    const bool torque_power_bad =
        (abs_err > cfg.torque_power_abs_tol) &&
        (rel_err > cfg.torque_power_rel_tol);

    if (torque_power_bad) {
        out.code = ErrorCode::NumericalFailure;
        out.message = "power mismatch vs Q*omega";
        return out;
    }

    out.code = ErrorCode::Ok;
    out.message = "ok";
    return out;
}

} // namespace lift::bemt
