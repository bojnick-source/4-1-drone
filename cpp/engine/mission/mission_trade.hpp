/*
===============================================================================
Fragment 3.1.32 — Mission Scoring Impacts (Time-to-Complete vs Mass/Energy Trade, Deterministic) (C++)
File: cpp/engine/mission/mission_trade.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstdint>
#include <string>

namespace lift::mission {

inline double clamp01(double x) noexcept {
  if (!lift::bemt::is_finite(x)) return 0.0;
  if (x < 0.0) return 0.0;
  if (x > 1.0) return 1.0;
  return x;
}

// Smooth-ish monotonic normalizer for positive values
// norm(x, x_ref) = x / (x + x_ref)  in [0,1)
inline double norm_pos(double x, double x_ref) noexcept {
  if (!lift::bemt::is_finite(x) || x < 0.0) x = 0.0;
  if (!lift::bemt::is_finite(x_ref) || x_ref <= 0.0) x_ref = 1.0;
  return clamp01(x / (x + x_ref));
}

// For “lower is better” metrics (time, energy, mass):
// benefit = 1 - norm_pos(x, x_ref)  (higher is better)
inline double benefit_lower_better(double x, double x_ref) noexcept {
  return clamp01(1.0 - norm_pos(x, x_ref));
}

// For “higher is better” metrics (payload fraction):
inline double benefit_higher_better(double x, double x_ref) noexcept {
  // Using same bounded mapping: x/(x+x_ref)
  return clamp01(norm_pos(x, x_ref));
}

struct MissionTradeConfig final {
  // Reference values to normalize against (tuned per scenario)
  double time_ref_s = 60.0;
  double energy_ref_Wh = 1000.0;
  double mass_ref_kg = 25.0;
  double payload_ref_kg = 100.0;

  // Weights (sum not required but recommended)
  double w_time = 0.40;
  double w_energy = 0.30;
  double w_mass = 0.20;
  double w_payload = 0.10;

  // Optional constraint penalty (applied when rules/gates fail)
  double fail_penalty = 1.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(time_ref_s) && time_ref_s > 0.0, lift::bemt::ErrorCode::InvalidConfig, "time_ref_s invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(energy_ref_Wh) && energy_ref_Wh > 0.0, lift::bemt::ErrorCode::InvalidConfig, "energy_ref_Wh invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(mass_ref_kg) && mass_ref_kg > 0.0, lift::bemt::ErrorCode::InvalidConfig, "mass_ref_kg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(payload_ref_kg) && payload_ref_kg > 0.0, lift::bemt::ErrorCode::InvalidConfig, "payload_ref_kg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(w_time) && lift::bemt::is_finite(w_energy) && lift::bemt::is_finite(w_mass) && lift::bemt::is_finite(w_payload),
                      lift::bemt::ErrorCode::InvalidConfig, "weights invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(fail_penalty) && fail_penalty >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "fail_penalty invalid");
  }
};

struct MissionMetrics final {
  double time_s = 0.0;
  double energy_Wh = 0.0;
  double empty_mass_kg = 0.0;
  double payload_kg = 0.0;

  // Derived
  double payload_fraction = 0.0; // payload / (payload+empty)

  void sanitize() {
    if (!lift::bemt::is_finite(time_s) || time_s < 0.0) time_s = 0.0;
    if (!lift::bemt::is_finite(energy_Wh) || energy_Wh < 0.0) energy_Wh = 0.0;
    if (!lift::bemt::is_finite(empty_mass_kg) || empty_mass_kg < 0.0) empty_mass_kg = 0.0;
    if (!lift::bemt::is_finite(payload_kg) || payload_kg < 0.0) payload_kg = 0.0;

    const double gross = empty_mass_kg + payload_kg;
    payload_fraction = (gross > 0.0) ? (payload_kg / gross) : 0.0;
    if (!lift::bemt::is_finite(payload_fraction) || payload_fraction < 0.0) payload_fraction = 0.0;
  }
};

struct MissionTradeOut final {
  // Component benefits in [0,1]
  double b_time = 0.0;
  double b_energy = 0.0;
  double b_mass = 0.0;
  double b_payload = 0.0;

  // Weighted sum in [0, ~1]
  double score = 0.0;

  // Penalties
  double penalty = 0.0;

  double final_score = 0.0;
};

inline MissionTradeOut evaluate_trade(const MissionMetrics& m_in,
                                      const MissionTradeConfig& cfg_in,
                                      bool gates_pass = true) {
  MissionTradeConfig cfg = cfg_in;
  cfg.validate();

  MissionMetrics m = m_in;
  m.sanitize();

  MissionTradeOut out;
  out.b_time = benefit_lower_better(m.time_s, cfg.time_ref_s);
  out.b_energy = benefit_lower_better(m.energy_Wh, cfg.energy_ref_Wh);
  out.b_mass = benefit_lower_better(m.empty_mass_kg, cfg.mass_ref_kg);
  // payload benefit based on payload fraction by default (more aligned with “ratio”)
  out.b_payload = benefit_higher_better(m.payload_fraction, 0.5); // ref=0.5 makes 50% payload fraction mid-scale

  const double wsum = cfg.w_time + cfg.w_energy + cfg.w_mass + cfg.w_payload;
  const double inv = (wsum > 0.0 ? (1.0 / wsum) : 1.0);

  out.score = inv * (cfg.w_time * out.b_time +
                     cfg.w_energy * out.b_energy +
                     cfg.w_mass * out.b_mass +
                     cfg.w_payload * out.b_payload);

  if (!lift::bemt::is_finite(out.score) || out.score < 0.0) out.score = 0.0;

  out.penalty = gates_pass ? 0.0 : cfg.fail_penalty;
  out.final_score = std::max(0.0, out.score - out.penalty);
  return out;
}

} // namespace lift::mission
