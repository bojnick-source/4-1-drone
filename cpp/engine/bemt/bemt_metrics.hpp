/*
===============================================================================
Fragment 3.1.56 — BEMT Metrics Registry + Evidence Export Helpers (Closeout-Ready) (C++)
File: cpp/engine/bemt/bemt_metrics.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_core.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include "../compliance/rules_verification.hpp"
#include "../closeout/closeout_pipeline.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace lift::bemt::metrics {

// Canonical metric IDs
inline constexpr const char* kThrustN = "BEMT.THRUST_N";
inline constexpr const char* kTorqueNm = "BEMT.TORQUE_NM";
inline constexpr const char* kPowerW = "BEMT.POWER_W";
inline constexpr const char* kCt = "BEMT.CT";
inline constexpr const char* kCq = "BEMT.CQ";
inline constexpr const char* kCp = "BEMT.CP";
inline constexpr const char* kFM = "BEMT.FM";
inline constexpr const char* kPropEff = "BEMT.PROP_EFF";
inline constexpr const char* kResidual = "BEMT.RESIDUAL";
inline constexpr const char* kIters = "BEMT.ITERS";
inline constexpr const char* kDiskAreaM2 = "BEMT.DISK_AREA_M2";
inline constexpr const char* kTipSpeedMS = "BEMT.TIP_SPEED_M_S";
inline constexpr const char* kRPM = "BEMT.RPM";
inline constexpr const char* kDiskLoadingNm2 = "BEMT.DISK_LOADING_N_M2";
inline constexpr const char* kPidealW = "BEMT.P_IDEAL_INDUCED_W";
inline constexpr const char* kFMCheck = "BEMT.FM_CHECK";

inline double disk_area_m2(const RotorGeometry& g) noexcept {
  const double R = g.radius_m;
  if (!is_finite(R) || R <= 0.0) return 0.0;
  return lift::bemt::kPi * R * R;
}

inline double tip_speed_m_s(const RotorGeometry& g, const OperatingPoint& op) noexcept {
  if (!is_finite(op.omega_rad_s) || op.omega_rad_s <= 0.0) return 0.0;
  const double R = g.radius_m;
  if (!is_finite(R) || R <= 0.0) return 0.0;
  const double v = op.omega_rad_s * R;
  return is_finite(v) ? v : 0.0;
}

inline double rpm(const OperatingPoint& op) noexcept {
  if (!is_finite(op.omega_rad_s) || op.omega_rad_s <= 0.0) return 0.0;
  const double r = op.omega_rad_s * 60.0 / (2.0 * lift::bemt::kPi);
  return is_finite(r) ? r : 0.0;
}

inline double disk_loading_N_m2(double thrust_N, const RotorGeometry& g) noexcept {
  const double A = disk_area_m2(g);
  if (!is_finite(thrust_N) || thrust_N <= 0.0) return 0.0;
  if (!is_finite(A) || A <= 0.0) return 0.0;
  const double dl = thrust_N / A;
  return is_finite(dl) ? dl : 0.0;
}

inline double ideal_induced_power_W(double thrust_N, const Environment& env, const RotorGeometry& g) noexcept {
  const double rho = env.rho;
  const double A = disk_area_m2(g);
  if (!is_finite(thrust_N) || thrust_N <= 0.0) return 0.0;
  if (!is_finite(rho) || rho <= 0.0) return 0.0;
  if (!is_finite(A) || A <= 0.0) return 0.0;
  const double p = std::pow(thrust_N, 1.5) / std::sqrt(2.0 * rho * A);
  return is_finite(p) ? p : 0.0;
}

inline double fm_check(double Pideal_W, double Pactual_W) noexcept {
  if (!is_finite(Pideal_W) || Pideal_W <= 0.0) return 0.0;
  if (!is_finite(Pactual_W) || Pactual_W <= 0.0) return 0.0;
  const double fm = Pideal_W / Pactual_W;
  return is_finite(fm) ? fm : 0.0;
}

inline void add_evidence(std::vector<lift::compliance::EvidenceItem>& ev,
                         const std::string& key,
                         double value,
                         const std::string& unit,
                         const std::string& source) {
  if (!lift::bemt::is_finite(value)) return;
  lift::compliance::EvidenceItem e;
  e.key = key;
  e.value = value;
  e.unit = unit;
  e.source = source;
  e.validate();
  ev.push_back(std::move(e));
}

inline void add_kv(std::vector<lift::closeout::KV>& kv,
                   const std::string& key,
                   double value,
                   const std::string& unit,
                   const std::string& source) {
  if (!lift::bemt::is_finite(value)) return;
  lift::closeout::KV r;
  r.key = key;
  r.value = value;
  r.unit = unit;
  r.source = source;
  r.validate();
  kv.push_back(std::move(r));
}

inline void append_bemt_evidence(std::vector<lift::compliance::EvidenceItem>& ev,
                                 const RotorGeometry& geom,
                                 const Environment& env,
                                 const OperatingPoint& op,
                                 const BemtOutput& out,
                                 const std::string& source_prefix = "bemt") {
  const std::string src = source_prefix;

  add_evidence(ev, kThrustN, out.thrust_N, "N", src);
  add_evidence(ev, kTorqueNm, out.torque_Nm, "N*m", src);
  add_evidence(ev, kPowerW, out.power_W, "W", src);

  add_evidence(ev, kCt, out.Ct, "-", src);
  add_evidence(ev, kCq, out.Cq, "-", src);
  add_evidence(ev, kCp, out.Cp, "-", src);

  add_evidence(ev, kFM, out.FM, "-", src);
  add_evidence(ev, kPropEff, out.prop_eff, "-", src);
  add_evidence(ev, kResidual, out.residual, "-", src);
  add_evidence(ev, kIters, static_cast<double>(out.iters), "-", src);

  const double A = disk_area_m2(geom);
  const double Vtip = tip_speed_m_s(geom, op);
  const double rpm_v = rpm(op);
  const double DL = disk_loading_N_m2(out.thrust_N, geom);
  const double Pideal = ideal_induced_power_W(out.thrust_N, env, geom);
  const double FMc = fm_check(Pideal, out.power_W);

  add_evidence(ev, kDiskAreaM2, A, "m^2", "geometry");
  add_evidence(ev, kTipSpeedMS, Vtip, "m/s", src);
  add_evidence(ev, kRPM, rpm_v, "rpm", src);
  add_evidence(ev, kDiskLoadingNm2, DL, "N/m^2", src);
  add_evidence(ev, kPidealW, Pideal, "W", src);
  add_evidence(ev, kFMCheck, FMc, "-", src);
}

inline void append_bemt_kv(std::vector<lift::closeout::KV>& kv,
                           const RotorGeometry& geom,
                           const Environment& env,
                           const OperatingPoint& op,
                           const BemtOutput& out,
                           const std::string& source_prefix = "bemt") {
  const std::string src = source_prefix;

  add_kv(kv, kThrustN, out.thrust_N, "N", src);
  add_kv(kv, kTorqueNm, out.torque_Nm, "N*m", src);
  add_kv(kv, kPowerW, out.power_W, "W", src);

  add_kv(kv, kCt, out.Ct, "-", src);
  add_kv(kv, kCq, out.Cq, "-", src);
  add_kv(kv, kCp, out.Cp, "-", src);

  add_kv(kv, kFM, out.FM, "-", src);
  add_kv(kv, kPropEff, out.prop_eff, "-", src);
  add_kv(kv, kResidual, out.residual, "-", src);
  add_kv(kv, kIters, static_cast<double>(out.iters), "-", src);

  const double A = disk_area_m2(geom);
  const double Vtip = tip_speed_m_s(geom, op);
  const double rpm_v = rpm(op);
  const double DL = disk_loading_N_m2(out.thrust_N, geom);
  const double Pideal = ideal_induced_power_W(out.thrust_N, env, geom);
  const double FMc = fm_check(Pideal, out.power_W);

  add_kv(kv, kDiskAreaM2, A, "m^2", "geometry");
  add_kv(kv, kTipSpeedMS, Vtip, "m/s", src);
  add_kv(kv, kRPM, rpm_v, "rpm", src);
  add_kv(kv, kDiskLoadingNm2, DL, "N/m^2", src);
  add_kv(kv, kPidealW, Pideal, "W", src);
  add_kv(kv, kFMCheck, FMc, "-", src);
}

} // namespace lift::bemt::metrics

