// ============================================================================
// Fragment 3.1.12 — BEMT Solver Facade (Unified Hover/Forward Entry + Hardened Output + Diagnostics Hook) (C++)
// File: bemt_facade.hpp
// ============================================================================
//
// Purpose:
// - Provide a single, stable entry point for “run BEMT” used by:
//     * optimization loop (fast)
//     * Monte Carlo wrapper (uncertainty)
//     * closeout export (CSV)
// - This is a facade: it does NOT replace your existing bemt_solver/bemt_forward.
//   It wraps them, applies common safety/integrity/diagnostics, and standardizes returns.
//
// How to integrate:
// - Implement the two adapter functions at the bottom by calling your existing code:
//     bemt_run_hover_impl(...)
//     bemt_run_forward_impl(...)
// - Everything else here is hardened glue.
//
// ============================================================================

#pragma once
#include "bemt_error.hpp"
#include "bemt_num_limits.hpp"
#include "bemt_diagnostics.hpp"
#include "bemt_integrity.hpp"

#include <string>

namespace lift::bemt {

struct BemtRunIn final {
    // Minimal shared inputs (extend as needed)
    double rho_kg_m3 = 1.225;
    double disk_area_m2 = 0.0;

    double omega_rad_s = 0.0;

    // “Mode”: hover if forward_speed==0; forward otherwise (or use explicit switch)
    double forward_speed_m_s = 0.0;

    // Placeholder for your richer inputs:
    // - geometry pointer/handle
    // - airfoil tables registry handle
    // - rotor op (collective, inflow, etc.)
    void* user_geom = nullptr;
    void* user_airfoils = nullptr;
    void* user_op = nullptr;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(rho_kg_m3) && rho_kg_m3 > 0.0, ErrorCode::InvalidInput, "BemtRunIn.rho invalid");
        LIFT_BEMT_REQUIRE(is_finite(disk_area_m2) && disk_area_m2 >= 0.0, ErrorCode::InvalidInput, "BemtRunIn.disk_area invalid");
        LIFT_BEMT_REQUIRE(is_finite(omega_rad_s) && omega_rad_s >= 0.0, ErrorCode::InvalidInput, "BemtRunIn.omega invalid");
        LIFT_BEMT_REQUIRE(is_finite(forward_speed_m_s) && forward_speed_m_s >= 0.0, ErrorCode::InvalidInput, "BemtRunIn.forward_speed invalid");
    }
};

struct BemtRunOut final {
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    // Totals
    double T_N = 0.0;
    double Q_Nm = 0.0;
    double P_W = 0.0;

    // Derived
    double FM = 0.0;
    double eta = 0.0;
    double disk_loading_N_m2 = 0.0;

    // Diagnostics summary (optional)
    BemtDiagnostics diag;

    bool ok() const noexcept { return code == ErrorCode::Ok; }
};

// -----------------------------
// Adapters you implement (CALL YOUR EXISTING SOLVERS)
// -----------------------------

// Must fill T_N, Q_Nm, P_W, optional FM.
// Return ErrorCode::Ok on success; otherwise a failure code.
ErrorCode bemt_run_hover_impl(const BemtRunIn& in, BemtRunOut& out);

// Must fill T_N, Q_Nm, P_W, optional eta.
// Return ErrorCode::Ok on success; otherwise a failure code.
ErrorCode bemt_run_forward_impl(const BemtRunIn& in, BemtRunOut& out);

// -----------------------------
// Facade: unified run
// -----------------------------

struct BemtFacadeConfig final {
    BemtNumLimits limits{};
    IntegrityConfig integrity{};
    bool attach_diagnostics = true;
    bool run_integrity = true;

    void validate() const {
        limits.validate();
        integrity.validate();
    }
};

inline BemtRunOut bemt_run(const BemtRunIn& in, const BemtFacadeConfig& cfg_in = {}) {
    BemtFacadeConfig cfg = cfg_in;
    cfg.validate();
    in.validate();

    BemtRunOut out;
    out.diag = BemtDiagnostics{};
    out.diag.code = ErrorCode::Ok;

    // Run underlying solver
    ErrorCode rc = ErrorCode::Ok;
    if (in.forward_speed_m_s <= 0.0) {
        rc = bemt_run_hover_impl(in, out);
    } else {
        rc = bemt_run_forward_impl(in, out);
    }

    out.code = rc;
    out.message = (rc == ErrorCode::Ok) ? "ok" : "solver failed";

    // Postpass clamps
    if (cfg.attach_diagnostics) {
        clamp_postpass(out.T_N, out.Q_Nm, out.P_W, cfg.limits, out.diag);
    } else {
        bool c = false;
        out.T_N = clamp_thrust(out.T_N, cfg.limits, c);
        out.Q_Nm = clamp_torque(out.Q_Nm, cfg.limits, c);
        out.P_W = clamp_power(out.P_W, cfg.limits, c);
    }

    // Derived disk loading
    if (in.disk_area_m2 > 0.0) {
        out.disk_loading_N_m2 = out.T_N / in.disk_area_m2;
        if (!is_finite(out.disk_loading_N_m2) || out.disk_loading_N_m2 < 0.0) out.disk_loading_N_m2 = 0.0;
    }

    // Integrity checks (maps failures into diag + error codes)
    if (cfg.run_integrity && cfg.attach_diagnostics) {
        attach_integrity(out.diag, out.T_N, out.Q_Nm, out.P_W, in.rho_kg_m3, in.disk_area_m2, in.omega_rad_s, cfg.integrity);
        if (!out.diag.ok()) {
            out.code = out.diag.code;
            out.message = out.diag.reason.empty() ? "integrity failure" : out.diag.reason;
        } else {
            out.FM = out.diag.fm; // recomputed FM (hover) if area available
        }
    }

    return out;
}

} // namespace lift::bemt
