#include "engine/physics/bemt_sensitivity.hpp"

#include <cmath>

namespace lift::bemt {

namespace {

inline double rel_step(double base, double rel) noexcept {
  const double mag = std::abs(base);
  const double s = (mag > 1e-12) ? (mag * rel) : rel;
  return (is_finite(s) && s > 0.0) ? s : rel;
}

}  // namespace

RotorGeometry SensitivityAnalyzer::scale_geom_(const RotorGeometry& g, double radius_scale, double chord_scale) {
  RotorGeometry out = g;
  radius_scale = positive_or(radius_scale, 1.0);
  chord_scale = positive_or(chord_scale, 1.0);

  out.radius_m *= radius_scale;
  out.hub_radius_m *= radius_scale;
  for (auto& st : out.stations) {
    st.r_m *= radius_scale;
    st.chord_m *= chord_scale;
  }
  return out;
}

NormalizedSens SensitivityAnalyzer::norm_from_(double x, double T0, double P0, double dTdx, double dPdx) noexcept {
  NormalizedSens n{};
  n.n_dT = safe_div(x * dTdx, T0, 0.0);
  n.n_dP = safe_div(x * dPdx, P0, 0.0);
  if (!is_finite(n.n_dT)) n.n_dT = 0.0;
  if (!is_finite(n.n_dP)) n.n_dP = 0.0;
  return n;
}

BemtResult SensitivityAnalyzer::solve_(BemtInputs in, bool allow_trim) const {
  if (!allow_trim) in.op.target_thrust_N.reset();
  return solver_.solve(in);
}

SensitivityResult SensitivityAnalyzer::compute(const BemtInputs& in, const SensitivityConfig& cfg) const {
  in.geom.validate();
  in.env.validate();
  in.op.validate();
  in.cfg.validate();
  cfg.validate();

  SensitivityResult out{};

  // Baseline
  const BemtResult base = solve_(in, cfg.allow_trim);
  out.code = base.code;
  if (base.code != ErrorCode::Ok) return out;

  const double T0 = base.thrust_N;
  const double P0 = base.power_W;

  // Omega
  {
    const double step = rel_step(in.op.omega_rad_s, cfg.h_omega_rel);
    BemtInputs ip = in;
    ip.op.omega_rad_s += step;

    BemtInputs im = in;
    if (cfg.central_difference) im.op.omega_rad_s = std::max(1e-6, in.op.omega_rad_s - step);

    const BemtResult rp = solve_(ip, cfg.allow_trim);
    if (rp.code != ErrorCode::Ok) { out.code = rp.code; return out; }

    BemtResult rm = base;
    if (cfg.central_difference) {
      rm = solve_(im, cfg.allow_trim);
      if (rm.code != ErrorCode::Ok) { out.code = rm.code; return out; }
    }

    const double dTdx = cfg.central_difference ? (rp.thrust_N - rm.thrust_N) / (2.0 * step)
                                                : (rp.thrust_N - T0) / step;
    const double dPdx = cfg.central_difference ? (rp.power_W - rm.power_W) / (2.0 * step)
                                                : (rp.power_W - P0) / step;
    out.omega = norm_from_(in.op.omega_rad_s, T0, P0, dTdx, dPdx);
  }

  // Density
  {
    const double step = rel_step(in.env.rho, cfg.h_rho_rel);
    BemtInputs ip = in;
    ip.env.rho += step;

    BemtInputs im = in;
    if (cfg.central_difference) im.env.rho = std::max(1e-6, in.env.rho - step);

    const BemtResult rp = solve_(ip, cfg.allow_trim);
    if (rp.code != ErrorCode::Ok) { out.code = rp.code; return out; }

    BemtResult rm = base;
    if (cfg.central_difference) {
      rm = solve_(im, cfg.allow_trim);
      if (rm.code != ErrorCode::Ok) { out.code = rm.code; return out; }
    }

    const double dTdx = cfg.central_difference ? (rp.thrust_N - rm.thrust_N) / (2.0 * step)
                                                : (rp.thrust_N - T0) / step;
    const double dPdx = cfg.central_difference ? (rp.power_W - rm.power_W) / (2.0 * step)
                                                : (rp.power_W - P0) / step;
    out.rho_sens = norm_from_(in.env.rho, T0, P0, dTdx, dPdx);
  }

  // Radius scale
  {
    const double step_rel = cfg.h_radius_rel;
    const double step = rel_step(1.0, step_rel);

    BemtInputs ip = in;
    ip.geom = scale_geom_(in.geom, 1.0 + step_rel, 1.0);

    BemtInputs im = in;
    if (cfg.central_difference) im.geom = scale_geom_(in.geom, std::max(0.1, 1.0 - step_rel), 1.0);

    const BemtResult rp = solve_(ip, cfg.allow_trim);
    if (rp.code != ErrorCode::Ok) { out.code = rp.code; return out; }

    BemtResult rm = base;
    if (cfg.central_difference) {
      rm = solve_(im, cfg.allow_trim);
      if (rm.code != ErrorCode::Ok) { out.code = rm.code; return out; }
    }

    const double dTdx = cfg.central_difference ? (rp.thrust_N - rm.thrust_N) / (2.0 * step)
                                                : (rp.thrust_N - T0) / step;
    const double dPdx = cfg.central_difference ? (rp.power_W - rm.power_W) / (2.0 * step)
                                                : (rp.power_W - P0) / step;
    out.radius_scale = norm_from_(1.0, T0, P0, dTdx, dPdx);
  }

  // Chord scale
  {
    const double step_rel = cfg.h_chord_rel;
    const double step = rel_step(1.0, step_rel);

    BemtInputs ip = in;
    ip.geom = scale_geom_(in.geom, 1.0, 1.0 + step_rel);

    BemtInputs im = in;
    if (cfg.central_difference) im.geom = scale_geom_(in.geom, 1.0, std::max(0.1, 1.0 - step_rel));

    const BemtResult rp = solve_(ip, cfg.allow_trim);
    if (rp.code != ErrorCode::Ok) { out.code = rp.code; return out; }

    BemtResult rm = base;
    if (cfg.central_difference) {
      rm = solve_(im, cfg.allow_trim);
      if (rm.code != ErrorCode::Ok) { out.code = rm.code; return out; }
    }

    const double dTdx = cfg.central_difference ? (rp.thrust_N - rm.thrust_N) / (2.0 * step)
                                                : (rp.thrust_N - T0) / step;
    const double dPdx = cfg.central_difference ? (rp.power_W - rm.power_W) / (2.0 * step)
                                                : (rp.power_W - P0) / step;
    out.chord_scale = norm_from_(1.0, T0, P0, dTdx, dPdx);
  }

  // Collective (absolute)
  {
    const double step = cfg.h_collective_abs_rad;
    BemtInputs ip = in;
    ip.op.collective_offset_rad += step;

    BemtInputs im = in;
    if (cfg.central_difference) im.op.collective_offset_rad -= step;

    const BemtResult rp = solve_(ip, cfg.allow_trim);
    if (rp.code != ErrorCode::Ok) { out.code = rp.code; return out; }

    BemtResult rm = base;
    if (cfg.central_difference) {
      rm = solve_(im, cfg.allow_trim);
      if (rm.code != ErrorCode::Ok) { out.code = rm.code; return out; }
    }

    const double dTdx = cfg.central_difference ? (rp.thrust_N - rm.thrust_N) / (2.0 * step)
                                                : (rp.thrust_N - T0) / step;
    const double dPdx = cfg.central_difference ? (rp.power_W - rm.power_W) / (2.0 * step)
                                                : (rp.power_W - P0) / step;
    out.collective = norm_from_(in.op.collective_offset_rad, T0, P0, dTdx, dPdx);
  }

  out.code = ErrorCode::Ok;
  return out;
}

}  // namespace lift::bemt
