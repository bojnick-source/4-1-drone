// ============================================================================
// Fragment 3.1.06 — BEMT Section Inflow Solve (Hover/Axial Station Solver, Bracketed Root-Find, Hardened) (C++)
// File: bemt_section_solver.hpp
// ============================================================================
//
// Purpose:
// - Hardened per-station blade-element + momentum closure for axial/hover-like conditions.
// - Solves for induced velocity vi >= 0 such that:
//     dT_BE(vi) == dT_MT(vi)
//   where:
//     dT_BE = 0.5*rho*Vrel^2 * B*c * (cl*cos(phi) - cd*sin(phi)) * dr
//     dT_MT = 4*pi*rho*F * r * vi * (va + vi) * dr
//     Vt    = omega*r
//     Va    = va + vi
//     phi   = atan2(Va, Vt)
//
// - Provides dQ (torque) from blade element (standard):
//     dQ_BE = 0.5*rho*Vrel^2 * B*c * r * (cl*sin(phi) + cd*cos(phi)) * dr
//
// Notes:
// - This is the “robust core primitive” you can call inside bemt_solver hover/forward,
//   even if your full solver later adds tangential induction a' and azimuthal integration.
// - This implementation focuses on speed + stability:
//     * bracketed solve (safe)
//     * conservative clamps
//     * explicit failure codes
//
// ============================================================================

#pragma once
#include "bemt_safety.hpp"
#include "bemt_kinematics.hpp"
#include "bemt_losses.hpp"

#include <cmath>
#include <cstddef>
#include <functional>
#include <string>

namespace lift::bemt {

struct SectionSolveConfig final {
    // Root-find iterations
    int max_iter = 60;
    double tol_rel = 1e-6;
    double tol_abs = 1e-6;

    // Induced velocity clamp (m/s)
    double vi_min = 0.0;
    double vi_max = 250.0;

    // If BE predicts negative thrust at vi=0, clamp to 0 and return NoThrust.
    bool clamp_negative_thrust = true;

    // Tip/root loss enable
    bool enable_tip_loss = true;
    bool enable_root_loss = true;

    void validate() const {
        LIFT_BEMT_REQUIRE(max_iter >= 10 && max_iter <= 400, ErrorCode::InvalidConfig, "SectionSolveConfig.max_iter invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_rel) && tol_rel > 0.0 && tol_rel < 1e-2, ErrorCode::InvalidConfig, "SectionSolveConfig.tol_rel invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_abs) && tol_abs > 0.0 && tol_abs < 1e2, ErrorCode::InvalidConfig, "SectionSolveConfig.tol_abs invalid");
        LIFT_BEMT_REQUIRE(is_finite(vi_min) && is_finite(vi_max) && vi_min >= 0.0 && vi_max > vi_min,
                          ErrorCode::InvalidConfig, "SectionSolveConfig.vi bounds invalid");
    }
};

// Airfoil evaluation callback:
// Inputs: alpha_rad, Re -> outputs: cl, cd
using AirfoilEvalFn = std::function<void(double alpha_rad, double Re, double& cl, double& cd)>;

struct SectionSolveIn final {
    // Air state
    double rho = 1.225;
    double mu  = 1.81e-5;

    // Rotor/section geometry
    int blades = 2;
    double r_m = 0.0;
    double dr_m = 0.0;
    double Rhub_m = 0.0;
    double Rtip_m = 0.0;
    double chord_m = 0.0;
    double twist_rad = 0.0;

    // Operating point
    double omega_rad_s = 0.0;
    double collective_rad = 0.0;

    // Axial free-stream through disk (m/s). Hover: 0.
    // Sign convention: treated as magnitude in atan2; keep consistent within your solver.
    double v_axial_m_s = 0.0;

    // Airfoil evaluator (must be provided)
    AirfoilEvalFn airfoil_eval;

    void validate() const {
        require_positive(rho, ErrorCode::InvalidInput, "SectionSolveIn.rho invalid");
        require_positive(mu,  ErrorCode::InvalidInput, "SectionSolveIn.mu invalid");
        LIFT_BEMT_REQUIRE(blades >= 2 && blades <= 12, ErrorCode::InvalidInput, "SectionSolveIn.blades invalid");

        require_nonneg(r_m,  ErrorCode::InvalidInput, "SectionSolveIn.r invalid");
        require_nonneg(dr_m, ErrorCode::InvalidInput, "SectionSolveIn.dr invalid");
        require_nonneg(Rhub_m, ErrorCode::InvalidInput, "SectionSolveIn.Rhub invalid");
        require_positive(Rtip_m, ErrorCode::InvalidInput, "SectionSolveIn.Rtip invalid");
        LIFT_BEMT_REQUIRE(Rtip_m > Rhub_m, ErrorCode::InvalidInput, "SectionSolveIn.Rtip must be > Rhub");
        LIFT_BEMT_REQUIRE(r_m >= Rhub_m && r_m <= Rtip_m, ErrorCode::InvalidInput, "SectionSolveIn.r out of [Rhub,Rtip]");

        require_positive(chord_m, ErrorCode::InvalidInput, "SectionSolveIn.chord invalid");
        require_finite(twist_rad, ErrorCode::InvalidInput, "SectionSolveIn.twist not finite");

        require_nonneg(omega_rad_s, ErrorCode::InvalidInput, "SectionSolveIn.omega invalid");
        require_finite(collective_rad, ErrorCode::InvalidInput, "SectionSolveIn.collective not finite");
        require_finite(v_axial_m_s, ErrorCode::InvalidInput, "SectionSolveIn.v_axial not finite");

        LIFT_BEMT_REQUIRE(static_cast<bool>(airfoil_eval), ErrorCode::InvalidInput, "SectionSolveIn.airfoil_eval missing");
    }
};

struct SectionSolveOut final {
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    // Solved induced velocity (m/s)
    double vi_m_s = 0.0;

    // Kinematics
    double Vt_m_s = 0.0;
    double Va_m_s = 0.0;
    double Vrel_m_s = 0.0;
    double phi_rad = 0.0;
    double alpha_rad = 0.0;
    double Re = 0.0;

    // Coeffs
    double cl = 0.0;
    double cd = 0.0;

    // Loss factor
    double F = 1.0;

    // Differential loads (TOTAL rotor contribution for this annulus)
    double dT_N = 0.0;
    double dQ_Nm = 0.0;

    // Residual at solution (N)
    double residual_N = 0.0;

    bool ok() const noexcept { return code == ErrorCode::Ok; }
};

// Compute BE thrust/torque at a given vi.
inline void section_be(const SectionSolveIn& in,
                       const SectionSolveConfig& cfg,
                       double vi,
                       SectionSolveOut& out) {
    // Effective velocities
    const double Vt = in.omega_rad_s * in.r_m;
    const double Va = in.v_axial_m_s + vi;

    BemtKinematicsIn kin;
    kin.rho_kg_m3 = in.rho;
    kin.mu_Pa_s   = in.mu;
    kin.omega_rad_s = in.omega_rad_s;
    kin.r_m = in.r_m;
    kin.chord_m = in.chord_m;
    kin.twist_rad = in.twist_rad;
    kin.collective_rad = in.collective_rad;
    kin.v_axial_m_s = Va;
    kin.v_inplane_m_s = 0.0;

    const auto k = bemt_kinematics(kin);

    // Tip/root loss (based on phi)
    const auto lf = prandtl_losses(in.blades, in.r_m, in.Rhub_m, in.Rtip_m, k.phi_rad,
                                   cfg.enable_tip_loss, cfg.enable_root_loss);

    // Airfoil coefficients
    double cl = 0.0, cd = 0.0;
    in.airfoil_eval(k.alpha_rad, (k.Re > 0.0 ? k.Re : 1.0e5), cl, cd);

    if (!is_finite(cl)) cl = 0.0;
    if (!is_finite(cd) || cd < 0.0) cd = 0.0;

    // Section normal/tangential force coefficients projected to thrust/torque
    const double cT = cl * std::cos(k.phi_rad) - cd * std::sin(k.phi_rad);
    const double cQ = cl * std::sin(k.phi_rad) + cd * std::cos(k.phi_rad);

    const double q = 0.5 * in.rho * (k.vrel_m_s * k.vrel_m_s);
    const double B = static_cast<double>(in.blades);

    // Differential loads (includes loss factor on thrust path)
    // Conservative: apply F to both thrust and torque.
    const double dT = q * B * in.chord_m * cT * in.dr_m * lf.F;
    const double dQ = q * B * in.chord_m * in.r_m * cQ * in.dr_m * lf.F;

    out.Vt_m_s = Vt;
    out.Va_m_s = Va;
    out.Vrel_m_s = k.vrel_m_s;
    out.phi_rad = k.phi_rad;
    out.alpha_rad = k.alpha_rad;
    out.Re = k.Re;
    out.cl = cl;
    out.cd = cd;
    out.F = lf.F;
    out.dT_N = (is_finite(dT) ? dT : 0.0);
    out.dQ_Nm = (is_finite(dQ) ? dQ : 0.0);
}

// Momentum thrust for annulus at a given vi (axial momentum, with Prandtl F)
inline double section_momentum_dT(const SectionSolveIn& in, double vi, double F) noexcept {
    // dT = 4*pi*rho*F*r*vi*(va+vi)*dr
    const double term = vi * (in.v_axial_m_s + vi);
    const double dT = 4.0 * kPi * in.rho * F * in.r_m * term * in.dr_m;
    if (!is_finite(dT)) return 0.0;
    return dT;
}

inline SectionSolveOut solve_section_axial(const SectionSolveIn& in,
                                          const SectionSolveConfig& cfg_in = {}) {
    SectionSolveConfig cfg = cfg_in;
    cfg.validate();
    in.validate();

    SectionSolveOut out;

    // Quick exits: degenerate dr or omega
    if (in.dr_m <= 0.0) {
        out.code = ErrorCode::InvalidInput;
        out.message = "dr_m <= 0";
        return out;
    }
    if (in.omega_rad_s <= 0.0 || in.r_m <= 0.0) {
        // No rotation => no meaningful BE thrust.
        out.code = ErrorCode::InvalidInput;
        out.message = "omega or r is zero";
        return out;
    }

    // Helper to compute residual f(vi) = dT_BE - dT_MT
    auto f = [&](double vi, SectionSolveOut& tmp) -> double {
        vi = clamp(vi, cfg.vi_min, cfg.vi_max);
        section_be(in, cfg, vi, tmp);
        const double dT_mt = section_momentum_dT(in, vi, tmp.F);
        const double res = tmp.dT_N - dT_mt;
        return is_finite(res) ? res : 0.0;
    };

    // Bracket the root.
    // vi_low = 0, vi_high grows until sign change or bounds reached.
    SectionSolveOut tL, tH;
    double vL = cfg.vi_min;
    double vH = std::max(cfg.vi_min + 1e-6, 1.0);

    double fL = f(vL, tL);

    // Negative thrust at vi=0: either clamp or return.
    if (fL < 0.0 && cfg.clamp_negative_thrust) {
        out = tL;
        out.vi_m_s = vL;
        out.residual_N = fL;
        out.dT_N = 0.0;
        out.dQ_Nm = 0.0;
        out.code = ErrorCode::InvalidInput;
        out.message = "negative thrust at vi=0; clamped";
        return out;
    }

    // Reasonable initial upper bound:
    // base on tangential speed scale
    const double Vt = in.omega_rad_s * in.r_m;
    const double vi_cap = clamp(2.0 * Vt + std::fabs(in.v_axial_m_s) + 5.0, cfg.vi_min + 1e-6, cfg.vi_max);

    bool bracketed = false;
    double fH = 0.0;

    // Expand vH geometrically up to vi_cap
    for (int k = 0; k < 20; ++k) {
        vH = std::min(vi_cap, (k == 0 ? vH : vH * 1.8));
        fH = f(vH, tH);

        if (fL == 0.0) { bracketed = true; vH = vL; fH = fL; break; }
        if ((fL > 0.0 && fH < 0.0) || (fL < 0.0 && fH > 0.0) || fH == 0.0) {
            bracketed = true;
            break;
        }
        if (vH >= vi_cap) break;
    }

    if (!bracketed) {
        // No sign change; return best-effort at vH (often means BE cannot meet momentum with this pitch).
        out = tH;
        out.vi_m_s = vH;
        out.residual_N = fH;
        out.code = ErrorCode::NonConverged;
        out.message = "no root bracket (BE and momentum did not cross)";
        return out;
    }

    // If exact at vL or vH
    if (std::fabs(fL) <= cfg.tol_abs) {
        out = tL; out.vi_m_s = vL; out.residual_N = fL; out.code = ErrorCode::Ok; out.message = "converged at vi_low";
        return out;
    }
    if (std::fabs(fH) <= cfg.tol_abs) {
        out = tH; out.vi_m_s = vH; out.residual_N = fH; out.code = ErrorCode::Ok; out.message = "converged at vi_high";
        return out;
    }

    // Bisection (robust). (Regula-falsi can be faster but less stable on flat residuals.)
    double a = vL, b = vH;
    double fa = fL, fb = fH;

    SectionSolveOut tM;
    for (int it = 0; it < cfg.max_iter; ++it) {
        const double m = 0.5 * (a + b);
        const double fm = f(m, tM);

        const double abs_e = std::fabs(fm);
        const double rel_e = abs_e / std::max(std::fabs(tM.dT_N), 1.0);

        if (abs_e <= cfg.tol_abs || rel_e <= cfg.tol_rel) {
            out = tM;
            out.vi_m_s = m;
            out.residual_N = fm;
            out.code = ErrorCode::Ok;
            out.message = "converged";
            return out;
        }

        // Maintain bracket
        if ((fa > 0.0 && fm < 0.0) || (fa < 0.0 && fm > 0.0)) {
            b = m; fb = fm;
        } else {
            a = m; fa = fm;
        }

        // Numerical guard
        if (!is_finite(a) || !is_finite(b) || !is_finite(fa) || !is_finite(fb)) {
            out = tM;
            out.vi_m_s = m;
            out.residual_N = fm;
            out.code = ErrorCode::NumericalFailure;
            out.message = "numerical failure during bisection";
            return out;
        }
    }

    // Max-iter
    const double m = 0.5 * (a + b);
    const double fm = f(m, tM);
    out = tM;
    out.vi_m_s = m;
    out.residual_N = fm;
    out.code = ErrorCode::NonConverged;
    out.message = "max_iter reached";
    return out;
}

} // namespace lift::bemt
