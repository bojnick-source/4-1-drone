/*
===============================================================================
Fragment 3.1.38 — Maneuverability Margins (Yaw Torque, Roll/Pitch Moments, Bandwidth Proxy, Turn-Radius Implications) (C++)
File: cpp/engine/controls/maneuverability.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::controls {

struct RotorAuthority final {
  std::string id;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double T_min_N = 0.0;
  double T_max_N = 0.0;
  double kQ_per_T = 0.0;
  double Q_min_Nm = 0.0;
  double Q_max_Nm = 0.0;
  int spin_dir = +1;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(x_m) && lift::bemt::is_finite(y_m) && lift::bemt::is_finite(z_m),
                      lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.pos invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(T_min_N) && T_min_N >= 0.0, lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.T_min invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(T_max_N) && T_max_N >= T_min_N, lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.T_max invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(kQ_per_T) && kQ_per_T >= 0.0, lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.kQ invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(Q_min_Nm) && Q_min_Nm >= 0.0, lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.Q_min invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(Q_max_Nm) && Q_max_Nm >= Q_min_Nm, lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.Q_max invalid");
    LIFT_BEMT_REQUIRE(spin_dir == +1 || spin_dir == -1, lift::bemt::ErrorCode::InvalidInput, "RotorAuthority.spin_dir invalid");
  }

  double Q_from_T(double T_N) const noexcept {
    if (kQ_per_T > 0.0) return kQ_per_T * std::max(0.0, T_N);
    const double q = 0.5 * (Q_min_Nm + Q_max_Nm);
    return q;
  }

  double Q_max_abs() const noexcept {
    if (kQ_per_T > 0.0) return kQ_per_T * std::max(0.0, T_max_N);
    return Q_max_Nm;
  }
};

struct InertiaDiag final {
  double Ixx = 0.0, Iyy = 0.0, Izz = 0.0;
  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(Ixx) && Ixx > 0.0, lift::bemt::ErrorCode::InvalidInput, "Ixx invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(Iyy) && Iyy > 0.0, lift::bemt::ErrorCode::InvalidInput, "Iyy invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(Izz) && Izz > 0.0, lift::bemt::ErrorCode::InvalidInput, "Izz invalid");
  }
};

struct ManeuverReq final {
  double yaw_moment_req_Nm = 0.0;
  double roll_moment_req_Nm = 0.0;
  double pitch_moment_req_Nm = 0.0;
  double a_lat_req_mps2 = 0.0;
  double mass_kg = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(yaw_moment_req_Nm) && yaw_moment_req_Nm >= 0.0, lift::bemt::ErrorCode::InvalidInput, "yaw req invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(roll_moment_req_Nm) && roll_moment_req_Nm >= 0.0, lift::bemt::ErrorCode::InvalidInput, "roll req invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(pitch_moment_req_Nm) && pitch_moment_req_Nm >= 0.0, lift::bemt::ErrorCode::InvalidInput, "pitch req invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(a_lat_req_mps2) && a_lat_req_mps2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "alat req invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(mass_kg) && mass_kg >= 0.0, lift::bemt::ErrorCode::InvalidInput, "mass invalid");
  }
};

struct ManeuverMetrics final {
  double yaw_moment_max_Nm = 0.0;
  double roll_moment_max_Nm = 0.0;
  double pitch_moment_max_Nm = 0.0;

  double yaw_margin = 0.0;
  double roll_margin = 0.0;
  double pitch_margin = 0.0;

  double yaw_alpha_max = 0.0;
  double roll_alpha_max = 0.0;
  double pitch_alpha_max = 0.0;

  double a_lat_max_mps2 = 0.0;
  double turn_radius_m = 0.0;
};

struct ManeuverConfig final {
  double thrust_headroom_frac = 0.15;
  double lateral_thrust_frac = 0.25;
  double V_turn_mps = 10.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(thrust_headroom_frac) && thrust_headroom_frac >= 0.0 && thrust_headroom_frac <= 0.5,
                      lift::bemt::ErrorCode::InvalidConfig, "thrust_headroom_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(lateral_thrust_frac) && lateral_thrust_frac >= 0.0 && lateral_thrust_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "lateral_thrust_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(V_turn_mps) && V_turn_mps >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "V_turn invalid");
  }
};

inline void estimate_roll_pitch_moments(const std::vector<RotorAuthority>& rotors,
                                        double headroom_frac,
                                        double& M_roll_max,
                                        double& M_pitch_max) {
  M_roll_max = 0.0;
  M_pitch_max = 0.0;

  for (const auto& r : rotors) {
    r.validate();
    const double dT = headroom_frac * r.T_max_N;
    if (dT <= 0.0) continue;
    M_roll_max += std::abs(dT * r.y_m);
    M_pitch_max += std::abs(dT * r.x_m);
  }

  if (!lift::bemt::is_finite(M_roll_max) || M_roll_max < 0.0) M_roll_max = 0.0;
  if (!lift::bemt::is_finite(M_pitch_max) || M_pitch_max < 0.0) M_pitch_max = 0.0;
}

inline double estimate_yaw_moment(const std::vector<RotorAuthority>& rotors) {
  double Qsum = 0.0;
  for (const auto& r : rotors) {
    r.validate();
    Qsum += r.Q_max_abs();
  }
  if (!lift::bemt::is_finite(Qsum) || Qsum < 0.0) Qsum = 0.0;
  return 0.5 * Qsum;
}

inline ManeuverMetrics compute_maneuverability(const std::vector<RotorAuthority>& rotors,
                                               const InertiaDiag& I_in,
                                               const ManeuverReq& req_in,
                                               const ManeuverConfig& cfg_in) {
  I_in.validate();
  req_in.validate();

  ManeuverConfig cfg = cfg_in;
  cfg.validate();

  ManeuverMetrics m{};

  estimate_roll_pitch_moments(rotors, cfg.thrust_headroom_frac, m.roll_moment_max_Nm, m.pitch_moment_max_Nm);
  m.yaw_moment_max_Nm = estimate_yaw_moment(rotors);

  if (req_in.yaw_moment_req_Nm > 0.0) m.yaw_margin = m.yaw_moment_max_Nm / req_in.yaw_moment_req_Nm;
  if (req_in.roll_moment_req_Nm > 0.0) m.roll_margin = m.roll_moment_max_Nm / req_in.roll_moment_req_Nm;
  if (req_in.pitch_moment_req_Nm > 0.0) m.pitch_margin = m.pitch_moment_max_Nm / req_in.pitch_moment_req_Nm;

  if (!lift::bemt::is_finite(m.yaw_margin) || m.yaw_margin < 0.0) m.yaw_margin = 0.0;
  if (!lift::bemt::is_finite(m.roll_margin) || m.roll_margin < 0.0) m.roll_margin = 0.0;
  if (!lift::bemt::is_finite(m.pitch_margin) || m.pitch_margin < 0.0) m.pitch_margin = 0.0;

  m.roll_alpha_max = m.roll_moment_max_Nm / I_in.Ixx;
  m.pitch_alpha_max = m.pitch_moment_max_Nm / I_in.Iyy;
  m.yaw_alpha_max = m.yaw_moment_max_Nm / I_in.Izz;

  if (!lift::bemt::is_finite(m.roll_alpha_max) || m.roll_alpha_max < 0.0) m.roll_alpha_max = 0.0;
  if (!lift::bemt::is_finite(m.pitch_alpha_max) || m.pitch_alpha_max < 0.0) m.pitch_alpha_max = 0.0;
  if (!lift::bemt::is_finite(m.yaw_alpha_max) || m.yaw_alpha_max < 0.0) m.yaw_alpha_max = 0.0;

  if (req_in.mass_kg > 0.0) {
    double Tsum = 0.0;
    for (const auto& r : rotors) Tsum += r.T_max_N;
    if (!lift::bemt::is_finite(Tsum) || Tsum < 0.0) Tsum = 0.0;

    m.a_lat_max_mps2 = (cfg.lateral_thrust_frac * Tsum) / req_in.mass_kg;
    if (!lift::bemt::is_finite(m.a_lat_max_mps2) || m.a_lat_max_mps2 < 0.0) m.a_lat_max_mps2 = 0.0;

    if (cfg.V_turn_mps > 0.0 && m.a_lat_max_mps2 > 0.0) {
      m.turn_radius_m = (cfg.V_turn_mps * cfg.V_turn_mps) / m.a_lat_max_mps2;
      if (!lift::bemt::is_finite(m.turn_radius_m) || m.turn_radius_m < 0.0) m.turn_radius_m = 0.0;
    }
  }

  return m;
}

} // namespace lift::controls
