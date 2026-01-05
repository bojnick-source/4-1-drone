/*
===============================================================================
Fragment 3.1.53 — BEMT Core Implementation (Hover + Forward, Induction Iteration, Tip Loss, Outputs/FM) (C++)
File: cpp/engine/physics/bemt_core.cpp
===============================================================================
*/

#include "engine/physics/bemt_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace lift::bemt {

static inline double sqr(double x) noexcept { return x * x; }
static inline double clamp01(double x) noexcept { return clamp(x, 0.0, 1.0); }

// Compute dr for station i using neighbor spacing; safe at ends.
static double station_dr(const RotorGeometry& g, std::size_t i) noexcept {
    const auto& st = g.stations;
    const std::size_t n = st.size();
    if (n < 2) return 0.0;

    if (i == 0) return std::max(0.0, st[1].r_m - st[0].r_m);
    if (i + 1 >= n) return std::max(0.0, st[n - 1].r_m - st[n - 2].r_m);

    const double dr1 = st[i].r_m - st[i - 1].r_m;
    const double dr2 = st[i + 1].r_m - st[i].r_m;
    const double dr = 0.5 * (dr1 + dr2);
    return std::max(0.0, dr);
}

// Prandtl tip loss factor (0..1)
static double prandtl_tip_loss(std::uint32_t B, double r, double R, double phi_rad, double min_F) noexcept {
    // Guard against phi near 0 (sin small) and r near 0.
    const double s = std::sin(std::abs(phi_rad));
    if (!is_finite(s) || s < 1e-6) return 1.0;
    if (!is_finite(r) || r <= 1e-9) return 1.0;
    if (!is_finite(R) || R <= r) return 1.0;

    const double f = (static_cast<double>(B) * 0.5) * ((R - r) / (r * s));
    if (!is_finite(f)) return 1.0;

    const double ex = std::exp(-f);
    const double arg = clamp(ex, 0.0, 1.0);
    const double F = (2.0 / kPi) * std::acos(arg);
    // Ensure not exactly 0 to avoid divide-by-zero in induction equations
    return clamp(F, min_F, 1.0);
}

// Computes nondimensional coefficients (optional)
static void compute_coeffs(BemtOutput& out, double rho, double A, double omega, double R) noexcept {
    if (!is_finite(rho) || !is_finite(A) || !is_finite(omega) || !is_finite(R)) return;
    if (rho <= 0.0 || A <= 0.0 || omega <= 0.0 || R <= 0.0) return;

    const double Vtip = omega * R;
    const double denomT = rho * A * sqr(Vtip);
    const double denomQ = rho * A * R * sqr(Vtip);
    const double denomP = rho * A * sqr(Vtip) * Vtip;

    if (denomT > 0.0) out.Ct = out.thrust_N / denomT;
    if (denomQ > 0.0) out.Cq = out.torque_Nm / denomQ;
    if (denomP > 0.0) out.Cp = out.power_W / denomP;
}

// Hover FM
static double compute_FM(double T, double P, double rho, double A) noexcept {
    if (!is_finite(T) || !is_finite(P) || !is_finite(rho) || !is_finite(A)) return 0.0;
    if (T <= 0.0 || P <= 0.0 || rho <= 0.0 || A <= 0.0) return 0.0;
    const double Pideal = std::pow(T, 1.5) / std::sqrt(2.0 * rho * A);
    if (!is_finite(Pideal) || Pideal <= 0.0) return 0.0;
    const double fm = Pideal / P;
    return clamp(fm, 0.0, 1.5); // allow slight >1 if upstream errors; clamp to conservative range
}

BemtOutput BemtCore::evaluate(const RotorGeometry& geom_in,
                              const IAirfoilDatabase& airfoils,
                              const Environment& env_in,
                              const OperatingPoint& op_in) const {
    BemtOutput out;
    out.code = ErrorCode::Ok;

    RotorGeometry geom = geom_in;
    Environment env = env_in;
    OperatingPoint op = op_in;

    geom.validate();
    env.validate();
    op.validate();
    cfg_.validate();

    const double rho = env.rho;
    const double mu  = env.mu;
    const double a_s = env.a_m_s;

    const double R = geom.radius_m;
    const double A = kPi * R * R;

    // Resolve freestream into axial component along rotor axis.
    const double Vz = op.V_inf * std::cos(op.inflow_angle_rad); // axial component

    const std::uint32_t B = static_cast<std::uint32_t>(geom.blade_count);
    const double omega = op.omega_rad_s;

    out.sections.clear();
    out.sections.reserve(geom.stations.size());

    double T = 0.0;
    double Q = 0.0;

    // Station-wise induction iteration
    double max_resid = 0.0;

    for (std::size_t i = 0; i < geom.stations.size(); ++i) {
        const auto& st = geom.stations[i];
        const double r = st.r_m;

        // Skip non-lifting region inside hub cutoff
        if (r < geom.hub_radius_m) continue;
        if (r <= 1e-6) continue;

        const double dr = station_dr(geom, i);
        if (dr <= 0.0) continue;

        // Local solidity
        const double sigma = (static_cast<double>(B) * st.chord_m) / (2.0 * kPi * r);

        // Initial guesses for induction
        double a_ind = 0.2;
        double a_tan = 0.0;

        // Fixed-point iterations
        double resid = 0.0;
        double phi = 0.0;
        double alpha = 0.0;
        double cl = 0.0, cd = 0.0;

        for (std::uint32_t it = 0; it < cfg_.max_iter; ++it) {
            // Effective velocities at blade element (deterministic, conservative):
            const double Vax = Vz * (1.0 + a_ind);
            const double Vtan = omega * r * (1.0 - a_tan);

            // Avoid degenerate
            const double Vtan_safe = (std::abs(Vtan) < 1e-9) ? (Vtan >= 0.0 ? 1e-9 : -1e-9) : Vtan;
            phi = std::atan2(Vax, Vtan_safe); // inflow angle

            // Apply collective + twist, compute effective alpha
            alpha = (st.twist_rad + op.collective_offset_rad) - phi;

            // Polar lookup
            PolarRequest preq;
            preq.airfoil_id = st.airfoil_id.empty() ? cfg_.default_airfoil_id : st.airfoil_id;
            preq.reynolds = 0.0; // set below with Re
            preq.mach = 0.0;

            PolarQuery pq;
            pq.aoa_rad = alpha;

            // Tip loss
            const bool use_prandtl = cfg_.use_prandtl_tip_loss && geom.tip_loss == TipLossModel::Prandtl;

            // Final local velocities for loads (depends on induction update)
            const double Vrel_tmp = std::sqrt(sqr(Vax) + sqr(Vtan_safe));
            const double Re_tmp = (mu > 0.0) ? (rho * Vrel_tmp * st.chord_m / mu) : 0.0;
            const double Ma_tmp = (a_s > 0.0) ? (Vrel_tmp / a_s) : 0.0;

            // Mach / Re safety checks before polar lookup
            if (cfg_.mach_max > 0.0 && is_finite(Ma_tmp) && Ma_tmp > cfg_.mach_max) {
                out.code = ErrorCode::OutOfRange;
                out.message = "Mach limit exceeded at r=" + std::to_string(r);
                return out;
            }
            if (cfg_.reynolds_min > 0.0 && is_finite(Re_tmp) && Re_tmp < cfg_.reynolds_min) {
                out.code = ErrorCode::OutOfRange;
                out.message = "Reynolds below min at r=" + std::to_string(r);
                return out;
            }
            if (cfg_.reynolds_max > 0.0 && is_finite(Re_tmp) && Re_tmp > cfg_.reynolds_max) {
                out.code = ErrorCode::OutOfRange;
                out.message = "Reynolds above max at r=" + std::to_string(r);
                return out;
            }

            pq.reynolds = Re_tmp;
            pq.mach = Ma_tmp;
            preq.reynolds = Re_tmp;
            preq.mach = Ma_tmp;

            std::shared_ptr<const IAirfoilPolar> polar;
            try {
                polar = airfoils.get_polar(preq);
            } catch (const std::exception& e) {
                out.code = ErrorCode::MissingPolarData;
                out.message = e.what();
                return out;
            }

            if (!polar) {
                out.code = ErrorCode::MissingPolarData;
                out.message = "no polar returned for airfoil";
                return out;
            }

            const PolarOutput po = polar->sample(pq);
            cl = po.cl;
            cd = po.cd;

            const double F = (use_prandtl ? prandtl_tip_loss(B, r, R, phi, cfg_.min_tip_loss_F) : 1.0);

            // Normal and tangential force coefficients in rotor coordinates
            const double Cn = cl * std::cos(phi) - cd * std::sin(phi);
            const double Ct = cl * std::sin(phi) + cd * std::cos(phi);

            // Guard: if Cn is too small, induction update becomes unstable
            const double Cn_safe = (std::abs(Cn) < 1e-9) ? (Cn >= 0.0 ? 1e-9 : -1e-9) : Cn;
            const double Ct_safe = (std::abs(Ct) < 1e-9) ? (Ct >= 0.0 ? 1e-9 : -1e-9) : Ct;

            // Induction update (standard BEMT form; conservative clamps):
            const double sphi = std::sin(phi);
            const double cphi = std::cos(phi);

            const double denom_a  = (4.0 * F * sphi * sphi) / (sigma * Cn_safe) + 1.0;
            const double denom_ap = (4.0 * F * sphi * cphi) / (sigma * Ct_safe) - 1.0;

            double a_new = a_ind;
            double ap_new = a_tan;

            if (is_finite(denom_a) && std::abs(denom_a) > 1e-9) a_new = 1.0 / denom_a;
            if (is_finite(denom_ap) && std::abs(denom_ap) > 1e-9) ap_new = 1.0 / denom_ap;

            // Clamp induction to stable bounds (prevents runaway in optimizer loops)
            a_new  = clamp(a_new,  -0.2,  0.95);
            ap_new = clamp(ap_new, -0.5,  0.5);

            // Relaxation for convergence
            const double a_rel  = a_ind + cfg_.relaxation * (a_new  - a_ind);
            const double ap_rel = a_tan + cfg_.relaxation * (ap_new - a_tan);

            // Residual
            resid = std::max(std::abs(a_rel - a_ind), std::abs(ap_rel - a_tan));

            a_ind = a_rel;
            a_tan = ap_rel;

            if (resid < cfg_.tol) {
                out.iters = std::max(out.iters, static_cast<std::size_t>(it + 1));
                break;
            }
            out.iters = std::max(out.iters, static_cast<std::size_t>(it + 1));
        }

        max_resid = std::max(max_resid, resid);

        // Final local velocities for loads
        const double Vax = Vz * (1.0 + a_ind);
        const double Vtan = omega * r * (1.0 - a_tan);
        const double Vrel = std::sqrt(sqr(Vax) + sqr(Vtan));

        // Reynolds and Mach
        const double Re = (mu > 0.0) ? (rho * Vrel * st.chord_m / mu) : 0.0;
        const double Ma = (a_s > 0.0) ? (Vrel / a_s) : 0.0;

        // Aerodynamic forces per unit span
        const double q = 0.5 * rho * sqr(Vrel);
        const double L_per_m = q * st.chord_m * cl;
        const double D_per_m = q * st.chord_m * cd;

        // Resolve to thrust/torque contributions
        const double dT = static_cast<double>(B) * (L_per_m * std::cos(phi) - D_per_m * std::sin(phi)) * dr;
        const double dQ = static_cast<double>(B) * (L_per_m * std::sin(phi) + D_per_m * std::cos(phi)) * r * dr;

        if (!is_finite(dT) || !is_finite(dQ)) {
            out.code = ErrorCode::NumericalError;
            out.message = "non-finite sectional loads at r=" + std::to_string(r);
            return out;
        }

        T += dT;
        Q += dQ;

        SectionOutput so;
        so.r_m = r;
        so.phi_rad = phi;
        so.alpha_rad = alpha;
        so.cl = cl;
        so.cd = cd;
        so.dT_N = dT;
        so.dQ_Nm = dQ;
        so.reynolds = Re;
        so.mach = Ma;
        out.sections.push_back(std::move(so));
    }

    out.thrust_N = T;
    out.torque_Nm = Q;
    out.power_W = std::max(0.0, Q * omega);
    out.residual = max_resid;

    // Coefficients (optional)
    compute_coeffs(out, rho, A, omega, R);

    // FM (hover only; interpret hover as V_inf ~ 0 and inflow_angle ~0)
    if (op.mode == OperatingPoint::FlightMode::Hover && std::abs(op.V_inf) <= 1e-6) {
        out.FM = compute_FM(out.thrust_N, out.power_W, rho, A);
    } else {
        out.FM = 0.0;
    }

    // Forward-flight prop efficiency proxy
    if (std::abs(op.V_inf) > 1e-9 && out.power_W > 1e-9) {
        // Use axial component for useful power proxy (conservative)
        const double V_use = std::max(0.0, Vz);
        out.prop_eff = clamp((out.thrust_N * V_use) / out.power_W, 0.0, 2.0);
    } else {
        out.prop_eff = 0.0;
    }

    // Convergence marking: if residual did not reach tol anywhere, keep Ok but message it
    if (cfg_.tol > 0.0 && max_resid > cfg_.tol) {
        out.message = "BEMT nonconverged: residual=" + std::to_string(max_resid);
    }

    return out;
}

} // namespace lift::bemt

