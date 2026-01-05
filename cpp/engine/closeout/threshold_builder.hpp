/*
===============================================================================
Fragment 3.1.62 — BEMT Scenario Threshold Builder (Mass→Thrust, Area, Power Budgets, Safety Margins) (C++)
File: cpp/engine/closeout/threshold_builder.hpp
===============================================================================
*/

#pragma once

#include "go_nogo_thresholds.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <cstdint>

namespace lift::closeout {

struct ScenarioSpec final {
  double aircraft_mass_kg = 0.0;
  double payload_mass_kg = 0.0;
  double g_m_s2 = 9.80665;
  std::uint32_t n_lift_rotors = 0;
  double thrust_margin = 1.0;
  double min_disk_area_m2 = 0.0;
  double max_power_W = 0.0;
  double max_residual = 1e-5;
  double min_FM = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(aircraft_mass_kg) && aircraft_mass_kg >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "aircraft_mass_kg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(payload_mass_kg) && payload_mass_kg >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "payload_mass_kg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(g_m_s2) && g_m_s2 > 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "g invalid");
    LIFT_BEMT_REQUIRE(n_lift_rotors >= 1, lift::bemt::ErrorCode::InvalidConfig, "n_lift_rotors < 1");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(thrust_margin) && thrust_margin >= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "thrust_margin invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_disk_area_m2) && min_disk_area_m2 >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "min_disk_area_m2 invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_power_W) && max_power_W >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "max_power_W invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_residual) && max_residual > 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "max_residual invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_FM) && min_FM >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "min_FM invalid");
  }
};

inline GoNoGoThresholds build_thresholds(const ScenarioSpec& s) {
  s.validate();

  const double gross_kg = s.aircraft_mass_kg + s.payload_mass_kg;
  const double gross_N = gross_kg * s.g_m_s2;
  const double per_rotor_N = (gross_N / static_cast<double>(s.n_lift_rotors)) * s.thrust_margin;

  GoNoGoThresholds t;
  t.min_thrust_N = per_rotor_N;
  t.max_power_W = (s.max_power_W > 0.0) ? s.max_power_W : 1.0e12;
  t.max_residual = s.max_residual;
  t.min_disk_area_m2 = s.min_disk_area_m2;
  t.min_FM = s.min_FM;
  t.validate();
  return t;
}

inline double payload_ratio(double aircraft_mass_kg, double payload_mass_kg) noexcept {
  if (!lift::bemt::is_finite(aircraft_mass_kg) || aircraft_mass_kg <= 0.0) return 0.0;
  if (!lift::bemt::is_finite(payload_mass_kg) || payload_mass_kg < 0.0) return 0.0;
  return payload_mass_kg / aircraft_mass_kg;
}

} // namespace lift::closeout

