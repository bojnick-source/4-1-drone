/*
===============================================================================
Fragment 3.1.10 — Hover BEMT Solver (C++)
File: bemt_solver.cpp
===============================================================================
*/

#include "engine/physics/bemt_solver.hpp"
#include "engine/physics/bemt_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lift::bemt {

namespace {
inline double sqr(double x) noexcept { return x * x; }
}  // namespace

double BemtSolver::station_dr_(const RotorGeometry& g, std::size_t i) noexcept {
    const auto& st = g.stations;
    const std::size_t n = st.size();
    if (n < 2) return 0.0;

    if (i == 0) return std::max(0.0, st[1].r_m - st[0].r_m);
    if (i + 1 >= n) return std::max(0.0, st[n - 1].r_m - st[n - 2].r_m);

    const double dr = 0.5 * ((st[i + 1].r_m - st[i].r_m) + (st[i].r_m - st[i - 1].r_m));
    return std::max(0.0, dr);
}

double BemtSolver::prandtl_tip_loss_(std::size_t B, double r, double R, double phi_rad) noexcept {
    if (B < 2) return 1.0;
    if (!(R > 0.0) || !(r > 0.0) || !(r < R)) return 1.0;

    const double s = std::sin(phi_rad);
    const double sabs = std::abs(s);
    if (!(sabs > 1e-6)) return 1.0;

    const double f = (static_cast<double>(B) / 2.0) * (R - r) / (r * sabs);
    if (!is_finite(f) || f <= 0.0) return 1.0;

    const double e = std::exp(-std::min(50.0, f));
    const double arg = clamp(e, 0.0, 1.0);

    const double F = (2.0 / kPi) * std::acos(arg);
    if (!is_finite(F)) return 1.0;
    return clamp(F, 0.05, 1.0);
}

double BemtSolver::induced_update_(double T, double rho, double A, double vax) noexcept {
    // Momentum closure:
    // T = 2*rho*A*vi*(vax + vi)
    // Solve quadratic: 2*rho*A*vi^2 + 2*rho*A*vax*vi - T = 0
    // vi = [-vax + sqrt(vax^2 + 2*T/(rho*A))]/2
    if (!is_finite(T) || T <= 0.0) return 0.0;
    if (!is_finite(rho) || rho <= 0.0) return 0.0;
    if (!is_finite(A) || A <= 0.0) return 0.0;
    if (!is_finite(vax)) vax = 0.0;

    const double disc = vax * vax + safe_div(2.0 * T, rho * A, 0.0);
    if (!is_finite(disc) || disc < 0.0) return 0.0;

    const double root = std::sqrt(disc);
    const double vi = 0.5 * (-vax + root);
    return (is_finite(vi) && vi >= 0.0) ? vi : 0.0;
}

BemtResult BemtSolver::solve_at_collective_(const RotorGeometry& g,
                                           const Environment& e,
                                           const OperatingPoint& op,
                                           const SolverConfig& cfg,
                                           double collective_offset_rad,
                                           double vi_init_mps) const {
    BemtResult out{};
    out.code = ErrorCode::Ok;
    out.collective_offset_rad = collective_offset_rad;
    out.stations.resize(g.stations.size());

    const double A = kPi * g.radius_m * g.radius_m;
    double vi = nonneg_or(vi_init_mps, 1.0);

    for (std::size_t it = 0; it < cfg.max_iter_inflow; ++it) {
        out.inflow_iters = it + 1;

        double T = 0.0;
        double Q = 0.0;

        for (std::size_t i = 0; i < g.stations.size(); ++i) {
            const BladeStation& bs = g.stations[i];

            StationResult sr{};
            sr.r_m = bs.r_m;
            sr.dr_m = std::max(cfg.min_dr_m, station_dr_(g, i));

            const double Vax = op.V_inf + vi;
            const double Vtan = op.omega_rad_s * bs.r_m;
            const double Vrel = std::sqrt(std::max(0.0, sqr(Vax) + sqr(Vtan)));

            sr.V_axial_m_s = Vax;
            sr.V_tan_m_s = Vtan;
            sr.V_rel_m_s = Vrel;

            const double phi = std::atan2(std::abs(Vax), std::max(1e-9, std::abs(Vtan)));
            sr.phi_rad = clamp(phi, cfg.min_phi_rad, cfg.max_phi_rad);

            const double theta = bs.twist_rad + collective_offset_rad;
            double aoa = theta - sr.phi_rad;
            aoa = clamp(aoa, cfg.min_aoa_rad, cfg.max_aoa_rad);
            sr.aoa_rad = aoa;

            sr.reynolds = safe_div(e.rho * Vrel * bs.chord_m, e.mu, 0.0);
            sr.mach = 0.0;

            const PolarOutput po = polar_.sample(PolarQuery{aoa, sr.reynolds, sr.mach});
            sr.cl = po.cl;
            sr.cd = po.cd;

            sr.tip_loss_F = 1.0;
            if (g.tip_loss == TipLossModel::Prandtl) {
                sr.tip_loss_F = prandtl_tip_loss_(g.blade_count, bs.r_m, g.radius_m, sr.phi_rad);
            }

            const double q = 0.5 * e.rho * Vrel * Vrel;
            const double Lp = q * bs.chord_m * sr.cl;
            const double Dp = q * bs.chord_m * sr.cd;

            const double dT_blade = (Lp * std::cos(sr.phi_rad) - Dp * std::sin(sr.phi_rad)) * sr.dr_m * sr.tip_loss_F;
            const double dQ_blade = (Lp * std::sin(sr.phi_rad) + Dp * std::cos(sr.phi_rad)) * bs.r_m * sr.dr_m * sr.tip_loss_F;

            sr.dT_N = dT_blade * static_cast<double>(g.blade_count);
            sr.dQ_Nm = dQ_blade * static_cast<double>(g.blade_count);

            if (is_finite(sr.dT_N)) T += sr.dT_N;
            if (is_finite(sr.dQ_Nm)) Q += sr.dQ_Nm;

            out.stations[i] = sr;
        }

        out.thrust_N = is_finite(T) ? std::max(0.0, T) : 0.0;
        out.torque_Nm = is_finite(Q) ? std::max(0.0, Q) : 0.0;
        out.power_W = out.torque_Nm * op.omega_rad_s;

        const double vi_new = induced_update_(out.thrust_N, e.rho, A, op.V_inf);
        const double err = std::abs(vi_new - vi);
        if (is_finite(err) && err <= cfg.tol_inflow) {
            vi = vi_new;
            out.induced_velocity_m_s = vi;
            out.figure_of_merit = figure_of_merit_from_rho(out.thrust_N, out.power_W, e.rho, A);
            return out;
        }

        const double vi_relaxed = (1.0 - cfg.inflow_relax) * vi + cfg.inflow_relax * vi_new;
        vi = clamp(vi_relaxed, 0.0, 250.0);
    }

    out.code = ErrorCode::NonConverged;
    out.induced_velocity_m_s = vi;
    out.figure_of_merit = 0.0;
    return out;
}

BemtResult BemtSolver::solve(const BemtInputs& in) const {
    in.geom.validate();
    in.env.validate();
    in.op.validate();
    in.cfg.validate();

    const RotorGeometry& g = in.geom;
    const Environment& e = in.env;
    const OperatingPoint& op = in.op;
    const SolverConfig& cfg = in.cfg;

    if (!op.target_thrust_N.has_value()) {
        return solve_at_collective_(g, e, op, cfg, op.collective_offset_rad, /*vi_init=*/2.0);
    }

    const double T_target = *op.target_thrust_N;

    double lo = cfg.collective_min_rad;
    double hi = cfg.collective_max_rad;

    double vi0 = 2.0;

    BemtResult r_lo = solve_at_collective_(g, e, op, cfg, lo, vi0);
    if (r_lo.code != ErrorCode::Ok) return r_lo;

    vi0 = r_lo.induced_velocity_m_s;

    BemtResult r_hi = solve_at_collective_(g, e, op, cfg, hi, vi0);
    if (r_hi.code != ErrorCode::Ok) return r_hi;

    auto f = [&](const BemtResult& r) { return r.thrust_N - T_target; };

    double flo = f(r_lo);
    double fhi = f(r_hi);

    if (flo * fhi > 0.0) {
        BemtResult best = (std::abs(flo) < std::abs(fhi)) ? r_lo : r_hi;
        best.code = ErrorCode::OutOfRange;
        return best;
    }

    double a = lo, b = hi;
    BemtResult ra = r_lo, rb = r_hi;

    for (std::size_t it = 0; it < cfg.max_iter_trim; ++it) {
        const double m = 0.5 * (a + b);
        const double vi_init = 0.5 * (ra.induced_velocity_m_s + rb.induced_velocity_m_s);

        BemtResult rm = solve_at_collective_(g, e, op, cfg, m, vi_init);
        rm.trim_iters = it + 1;

        if (rm.code != ErrorCode::Ok) return rm;

        const double fm = f(rm);
        if (std::abs(fm) <= cfg.tol_trim_N) {
            rm.code = ErrorCode::Ok;
            return rm;
        }

        const double fa = f(ra);
        if (fa * fm <= 0.0) {
            b = m;
            rb = rm;
        } else {
            a = m;
            ra = rm;
        }
    }

    BemtResult best = (std::abs(f(ra)) < std::abs(f(rb))) ? ra : rb;
    best.code = ErrorCode::NonConverged;
    return best;
}

} // namespace lift::bemt
