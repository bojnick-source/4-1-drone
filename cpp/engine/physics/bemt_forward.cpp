#include "engine/physics/bemt_forward.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lift::bemt {

static inline double sqr(double x) noexcept { return x * x; }

double BemtForwardSolver::station_dr_(const RotorGeometry& g, std::size_t i) noexcept {
  const auto& st = g.stations;
  const std::size_t n = st.size();
  if (n < 2) return 0.0;

  if (i == 0) return std::max(0.0, st[1].r_m - st[0].r_m);
  if (i + 1 >= n) return std::max(0.0, st[n - 1].r_m - st[n - 2].r_m);

  const double dr = 0.5 * ((st[i + 1].r_m - st[i].r_m) + (st[i].r_m - st[i - 1].r_m));
  return std::max(0.0, dr);
}

double BemtForwardSolver::prandtl_tip_loss_(std::size_t B, double r, double R, double phi_rad) noexcept {
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

ForwardResult BemtForwardSolver::solve(const RotorGeometry& g,
                                       const Environment& e,
                                       const OperatingPoint& op,
                                       const SolverConfig& scfg,
                                       double V_inplane_mps,
                                       const ForwardConfig& fcfg) const {
  g.validate();
  e.validate();
  op.validate();
  scfg.validate();
  fcfg.validate();

  LIFT_BEMT_REQUIRE(is_finite(V_inplane_mps) && V_inplane_mps >= 0.0 && V_inplane_mps < 250.0,
                    ErrorCode::InvalidInput, "V_inplane_mps invalid");

  ForwardResult out{};
  out.code = ErrorCode::Ok;

  const double A = kPi * g.radius_m * g.radius_m;
  LIFT_BEMT_REQUIRE(is_finite(A) && A > 0.0, ErrorCode::InvalidInput, "Rotor disk area invalid");

  // Initial induced velocity guess
  double vi = 2.0;

  // Azimuth step
  const double dpsi = (2.0 * kPi) / static_cast<double>(fcfg.n_psi);

  // Induced velocity iteration (swirl-free, scalar vi)
  for (std::size_t it = 0; it < fcfg.max_iter_vi; ++it) {
    out.vi_iters = it + 1;

    double T = 0.0;
    double Q = 0.0;

    // Integrate over azimuth and radius
    for (std::size_t i = 0; i < g.stations.size(); ++i) {
      const BladeStation& bs = g.stations[i];
      const double r = bs.r_m;
      const double dr = std::max(scfg.min_dr_m, station_dr_(g, i));

      for (std::size_t k = 0; k < fcfg.n_psi; ++k) {
        const double psi = (static_cast<double>(k) + 0.5) * dpsi;

        // In-plane forward component projected onto local tangential direction:
        // Vtan_eff = omega*r + Vin*cos(psi)  (advancing/retreating)
        // This is a standard low-order model for rotor forward flight.
        const double Vtan = op.omega_rad_s * r + V_inplane_mps * std::cos(psi);

        // Axial at disk (positive down): Vax = V_axial + vi
        const double Vax = fcfg.V_axial_mps + vi;

        const double Vrel = std::sqrt(std::max(0.0, Vax * Vax + Vtan * Vtan));
        const double phi = std::atan2(std::abs(Vax), std::max(1e-9, std::abs(Vtan)));

        const double theta = bs.twist_rad + op.collective_offset_rad;
        double aoa = theta - phi;

        const double phi_c = clamp(phi, fcfg.min_phi_rad, fcfg.max_phi_rad);
        aoa = clamp(aoa, scfg.min_aoa_rad, scfg.max_aoa_rad);

        const double Re = safe_div(e.rho * Vrel * bs.chord_m, e.mu, 0.0);
        const double mach = 0.0;

        const PolarOutput po = polar_.sample(PolarQuery{aoa, Re, mach});
        double cl = po.cl;
        double cd = po.cd;

        if (!is_finite(cl)) cl = 0.0;
        if (!is_finite(cd) || cd < 0.0) cd = 0.0;

        double F = 1.0;
        if (g.tip_loss == TipLossModel::Prandtl) {
          F = prandtl_tip_loss_(g.blade_count, r, g.radius_m, phi_c);
        }

        const double qdyn = 0.5 * e.rho * Vrel * Vrel;
        const double Lp = qdyn * bs.chord_m * cl;  // N/m
        const double Dp = qdyn * bs.chord_m * cd;  // N/m

        const double dT_blade = (Lp * std::cos(phi_c) - Dp * std::sin(phi_c)) * dr * F;
        const double dQ_blade = (Lp * std::sin(phi_c) + Dp * std::cos(phi_c)) * r * dr * F;

        const double dT = dT_blade * static_cast<double>(g.blade_count);
        const double dQ = dQ_blade * static_cast<double>(g.blade_count);

        if (is_finite(dT)) T += dT;
        if (is_finite(dQ)) Q += dQ;
      }
    }

    out.thrust_N = is_finite(T) ? std::max(0.0, T) : 0.0;
    out.torque_Nm = is_finite(Q) ? std::max(0.0, Q) : 0.0;
    out.power_W = out.torque_Nm * op.omega_rad_s;

    // Update induced velocity using swirl-free momentum surrogate
    const double Veff = std::sqrt(std::max(1e-12, sqr(fcfg.V_axial_mps + vi) + sqr(V_inplane_mps)));
    const double vi_new = safe_div(out.thrust_N, 2.0 * e.rho * A * Veff, 0.0);

    const double err = std::abs(vi_new - vi);
    if (is_finite(err) && err <= fcfg.tol_vi) {
      out.induced_velocity_mps = vi_new;
      return out;
    }

    const double vi_relaxed = (1.0 - fcfg.relax_vi) * vi + fcfg.relax_vi * vi_new;
    vi = std::max(0.0, vi_relaxed);
  }

  out.code = ErrorCode::NonConverged;
  out.induced_velocity_mps = vi;
  return out;
}

}  // namespace lift::bemt
