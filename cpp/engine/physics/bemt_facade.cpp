// ============================================================================
// Fragment 3.1.13 — BEMT Facade Adapter Stubs (Wire to Existing bemt_solver/bemt_forward) (C++)
// File: bemt_facade.cpp
// ============================================================================
//
// Purpose:
// - Provide the minimal implementation of the facade adapter functions declared in bemt_facade.hpp.
// - You MUST replace the “TODO: call your existing solver” sections with calls into your
//   current codebase (bemt_solver / bemt_forward / bemt_metrics).
//
// Contract:
// - Set out.T_N, out.Q_Nm, out.P_W (and optionally out.FM or out.eta).
// - Return ErrorCode::Ok on success.
// - Return a failure code on non-convergence/numerical failure, and fill out.message if desired.
//
// This file is intentionally small and auditable.
//
// ============================================================================

#include "bemt_facade.hpp"

// Include your existing solver headers here:
#include "bemt_solver.hpp"
#include "bemt_forward.hpp"
#include "bemt_metrics.hpp"

namespace lift::bemt {

static inline bool finite_all(double a, double b, double c) noexcept {
    return is_finite(a) && is_finite(b) && is_finite(c);
}

ErrorCode bemt_run_hover_impl(const BemtRunIn& in, BemtRunOut& out) {
    // TODO: Replace with your real types.
    // Example pattern (adapt to your actual interfaces):
    //
    // const auto* geom = static_cast<const RotorGeometry*>(in.user_geom);
    // const auto* air  = static_cast<const AirfoilRegistry*>(in.user_airfoils);
    // const auto* op   = static_cast<const RotorOp*>(in.user_op);
    //
    // if (!geom || !air || !op) return ErrorCode::InvalidInput;
    //
    // BemtSolver solver(*air);
    // const auto res = solver.solve_hover(*geom, *op, {in.rho_kg_m3, ...options...});
    // if (!res.ok()) return res.code;
    // out.T_N = res.T_N; out.Q_Nm = res.Q_Nm; out.P_W = res.P_W; out.FM = res.FM;
    //
    // return ErrorCode::Ok;

    (void)in;
    out.T_N = 0.0;
    out.Q_Nm = 0.0;
    out.P_W = 0.0;
    out.FM = 0.0;
    out.eta = 0.0;
    out.message = "bemt_run_hover_impl not wired";
    return ErrorCode::InvalidConfig;
}

ErrorCode bemt_run_forward_impl(const BemtRunIn& in, BemtRunOut& out) {
    // TODO: Replace with your real forward solver integration.
    //
    // Example:
    // BemtForward fwd(*air);
    // const auto res = fwd.solve(*geom, *op, in.forward_speed_m_s, air_state, options);
    // if (!res.ok()) return res.code;
    // out.T_N = res.T; out.Q_Nm = res.Q; out.P_W = res.P; out.eta = res.eta;
    // return ErrorCode::Ok;

    (void)in;
    out.T_N = 0.0;
    out.Q_Nm = 0.0;
    out.P_W = 0.0;
    out.FM = 0.0;
    out.eta = 0.0;
    out.message = "bemt_run_forward_impl not wired";
    return ErrorCode::InvalidConfig;
}

} // namespace lift::bemt
