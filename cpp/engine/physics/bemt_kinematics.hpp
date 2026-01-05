// ============================================================================
// Fragment 3.1.03 — BEMT Kinematics & Conventions (Phi/Alpha/Re + μ/λ + Safe Velocity) (C++)
// File: bemt_kinematics.hpp
// ============================================================================
//
// Purpose:
// - Centralize kinematics + sign conventions used by both hover and forward-flight BEMT.
// - Produce numerically safe quantities (phi, alpha, Re, nondimensional mu/lambda).
//
// Conventions (LOCK THESE):
// - omega_rad_s >= 0.
// - r_m in [r_hub, r_tip].
// - Tangential speed Vt = omega * r.
// - In-plane (forward) component is modeled as an additive tangential component magnitude:
//     V_inplane is treated as a magnitude added to local tangential speed for advance ratio.
//   (Full azimuthal integration is a later upgrade; this is low-order fast loop.)
//
// - Axial inflow sign: solver chooses; helpers just take magnitudes and return angles robustly.
//   Use va_m_s = V_axial_effective (positive means flow through disk along rotor axis).
//
// Outputs:
// - phi_rad: inflow angle = atan2(va, vt) (clamped, robust for vt~0).
// - alpha_rad: AoA = theta_rad - phi_rad (wrapped to (-pi, pi]) for stability.
// - Reynolds: Re = rho * V_rel * chord / mu.
//
// ============================================================================

#pragma once
#include "bemt_safety.hpp"

#include <cmath>
#include <limits>

namespace lift::bemt {

struct BemtKinematicsIn final {
    double rho_kg_m3 = 1.225;
    double mu_Pa_s   = 1.81e-5;

    double omega_rad_s = 0.0;
    double r_m = 0.0;
    double chord_m = 0.0;

    double twist_rad = 0.0;
    double collective_rad = 0.0;

    // Effective axial/in-plane velocities at this station (m/s).
    // Caller decides sign policy; helpers accept any finite values.
    double v_axial_m_s = 0.0;
    double v_inplane_m_s = 0.0;

    void validate() const {
        require_positive(rho_kg_m3, ErrorCode::InvalidInput, "BemtKinematicsIn.rho invalid");
        require_positive(mu_Pa_s,   ErrorCode::InvalidInput, "BemtKinematicsIn.mu invalid");

        require_nonneg(omega_rad_s, ErrorCode::InvalidInput, "BemtKinematicsIn.omega invalid");
        require_nonneg(r_m,         ErrorCode::InvalidInput, "BemtKinematicsIn.r invalid");
        require_positive(chord_m,   ErrorCode::InvalidInput, "BemtKinematicsIn.chord invalid");

        require_finite(twist_rad,     ErrorCode::InvalidInput, "BemtKinematicsIn.twist not finite");
        require_finite(collective_rad,ErrorCode::InvalidInput, "BemtKinematicsIn.collective not finite");

        require_finite(v_axial_m_s,   ErrorCode::InvalidInput, "BemtKinematicsIn.v_axial not finite");
        require_finite(v_inplane_m_s, ErrorCode::InvalidInput, "BemtKinematicsIn.v_inplane not finite");
        LIFT_BEMT_REQUIRE(v_inplane_m_s >= 0.0, ErrorCode::InvalidInput, "BemtKinematicsIn.v_inplane must be >=0 (magnitude)");
    }
};

struct BemtKinematicsOut final {
    double vt_m_s = 0.0;      // tangential speed
    double va_m_s = 0.0;      // axial speed (as provided)
    double vrel_m_s = 0.0;    // sqrt(vt^2 + va^2)

    double phi_rad = 0.0;     // atan2(va, vt)
    double theta_rad = 0.0;   // twist + collective
    double alpha_rad = 0.0;   // theta - phi (wrapped)

    double Re = 0.0;          // rho * Vrel * chord / mu

    // Nondimensional (using omega*R at *this r*):
    double mu_nd = 0.0;       // in-plane ratio ~ V_inplane/(omega*r) (low-order)
    double lambda_nd = 0.0;   // axial ratio ~ V_axial/(omega*r)

    void validate_basic() const {
        require_nonneg(vt_m_s,   ErrorCode::NumericalFailure, "BemtKinematicsOut.vt invalid");
        require_finite(va_m_s,   ErrorCode::NumericalFailure, "BemtKinematicsOut.va not finite");
        require_nonneg(vrel_m_s, ErrorCode::NumericalFailure, "BemtKinematicsOut.vrel invalid");
        require_finite(phi_rad,  ErrorCode::NumericalFailure, "BemtKinematicsOut.phi not finite");
        require_finite(theta_rad,ErrorCode::NumericalFailure, "BemtKinematicsOut.theta not finite");
        require_finite(alpha_rad,ErrorCode::NumericalFailure, "BemtKinematicsOut.alpha not finite");
        require_nonneg(Re,       ErrorCode::NumericalFailure, "BemtKinematicsOut.Re invalid");
        require_finite(mu_nd,    ErrorCode::NumericalFailure, "BemtKinematicsOut.mu_nd not finite");
        require_finite(lambda_nd,ErrorCode::NumericalFailure, "BemtKinematicsOut.lambda_nd not finite");
    }
};

inline BemtKinematicsOut bemt_kinematics(const BemtKinematicsIn& in) {
    in.validate();

    BemtKinematicsOut out;

    const double vt_rot = in.omega_rad_s * in.r_m;
    // Low-order forward model: treat in-plane speed as additive to tangential magnitude.
    const double vt = vt_rot + in.v_inplane_m_s;

    out.vt_m_s = (vt < 0.0 ? 0.0 : vt);
    out.va_m_s = in.v_axial_m_s;

    const double vt2 = out.vt_m_s * out.vt_m_s;
    const double va2 = out.va_m_s * out.va_m_s;
    out.vrel_m_s = safe_sqrt(vt2 + va2, 0.0);

    // Robust inflow angle
    out.phi_rad = safe_atan2(out.va_m_s, out.vt_m_s);

    out.theta_rad = in.twist_rad + in.collective_rad;
    out.alpha_rad = wrap_pi(out.theta_rad - out.phi_rad);

    // Reynolds: if Vrel==0, Re==0 (solver can handle; airfoil interp should clamp min Re)
    if (out.vrel_m_s > 0.0) {
        out.Re = (in.rho_kg_m3 * out.vrel_m_s * in.chord_m) / in.mu_Pa_s;
        if (!is_finite(out.Re) || out.Re < 0.0) out.Re = 0.0;
    } else {
        out.Re = 0.0;
    }

    // Nondimensional at this r:
    // guard omega*r ~ 0
    const double denom = std::max(in.omega_rad_s * in.r_m, 1e-12);
    out.mu_nd = in.v_inplane_m_s / denom;
    out.lambda_nd = in.v_axial_m_s / denom;

    // Sanity clamp extreme values (prevents runaway in solver control laws)
    out.mu_nd = clamp(out.mu_nd, 0.0, 5.0);
    out.lambda_nd = clamp(out.lambda_nd, -5.0, 5.0);

    out.validate_basic();
    return out;
}

} // namespace lift::bemt
