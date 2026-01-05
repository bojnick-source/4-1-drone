/*
===============================================================================
Fragment 3.1.41 — Mission Scoring Impact Closeout (Time-to-Complete vs Mass/Energy Trade) (C++)
File: cpp/engine/mission/mission_scoring.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::mission {

enum class SegmentType : std::uint8_t {
  Hover = 0,
  VerticalClimb = 1,
  Cruise = 2,
  Transition = 3,
  Descent = 4,
  Reserve = 5
};

struct Segment final {
  SegmentType type = SegmentType::Hover;
  double distance_m = 0.0;        // cruise distance or duration proxy
  double altitude_change_m = 0.0; // climb/descent magnitude
  double speed_mps = 0.0;
  double climb_rate_mps = 0.0;
  double descent_rate_mps = 0.0;
  double power_W = 0.0;           // override if >0

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(distance_m) && distance_m >= 0.0, lift::bemt::ErrorCode::InvalidInput, "Segment.distance invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(altitude_change_m) && std::abs(altitude_change_m) <= 1e6, lift::bemt::ErrorCode::InvalidInput, "Segment.alt_change invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(speed_mps) && speed_mps >= 0.0, lift::bemt::ErrorCode::InvalidInput, "Segment.speed invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(climb_rate_mps) && climb_rate_mps >= 0.0, lift::bemt::ErrorCode::InvalidInput, "Segment.climb_rate invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(descent_rate_mps) && descent_rate_mps >= 0.0, lift::bemt::ErrorCode::InvalidInput, "Segment.descent_rate invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(power_W) && power_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "Segment.power invalid");
  }
};

struct ScoringConfig final {
  double w_time = 1.0;
  double w_energy = 0.0;
  double w_mass = 0.0;
  double w_viol = 1e6;

  double max_total_time_s = 0.0; // <=0 disables
  double max_energy_J = 0.0;     // <=0 disables
  double max_mass_kg = 0.0;      // <=0 disables
  double max_power_W = 0.0;      // <=0 disables

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(w_time) && w_time >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "w_time invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(w_energy) && w_energy >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "w_energy invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(w_mass) && w_mass >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "w_mass invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(w_viol) && w_viol >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "w_viol invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_total_time_s) && max_total_time_s >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "max_total_time invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_energy_J) && max_energy_J >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "max_energy invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_mass_kg) && max_mass_kg >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "max_mass invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_power_W) && max_power_W >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "max_power invalid");
  }
};

struct MissionInputs final {
  double P_base_hover_W = 0.0;
  double P_base_cruise_W = 0.0;
  double dP_hover_W = 0.0;
  double dP_cruise_W = 0.0;
  double d_mass_kg = 0.0;
  double P_available_W = 0.0;
  std::vector<Segment> segments;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_base_hover_W) && P_base_hover_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "P_base_hover invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_base_cruise_W) && P_base_cruise_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "P_base_cruise invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(dP_hover_W), lift::bemt::ErrorCode::InvalidInput, "dP_hover invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(dP_cruise_W), lift::bemt::ErrorCode::InvalidInput, "dP_cruise invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(d_mass_kg), lift::bemt::ErrorCode::InvalidInput, "d_mass invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_available_W) && P_available_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "P_available invalid");
    LIFT_BEMT_REQUIRE(!segments.empty(), lift::bemt::ErrorCode::InvalidInput, "segments empty");
    for (const auto& s : segments) s.validate();
  }
};

struct SegmentResult final {
  SegmentType type{};
  double time_s = 0.0;
  double power_W = 0.0;
  double energy_J = 0.0;
  bool power_exceeded = false;
};

struct MissionResult final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
  std::vector<SegmentResult> segments;

  double total_time_s = 0.0;
  double total_energy_J = 0.0;

  double d_mass_kg = 0.0;
  double violations = 0.0;

  double score = 0.0; // lower is better
};

inline double segment_power_W(const Segment& s, const MissionInputs& in) noexcept {
  if (s.power_W > 0.0) return s.power_W;

  switch (s.type) {
    case SegmentType::Hover:
    case SegmentType::VerticalClimb:
    case SegmentType::Descent:
      return std::max(0.0, in.P_base_hover_W + in.dP_hover_W);
    case SegmentType::Cruise:
    case SegmentType::Transition:
    case SegmentType::Reserve:
    default:
      return std::max(0.0, in.P_base_cruise_W + in.dP_cruise_W);
  }
}

inline double segment_time_s(const Segment& s) noexcept {
  switch (s.type) {
    case SegmentType::Cruise:
      if (s.speed_mps > 0.0) return s.distance_m / s.speed_mps;
      return 0.0;
    case SegmentType::VerticalClimb:
      if (s.climb_rate_mps > 0.0) return std::abs(s.altitude_change_m) / s.climb_rate_mps;
      return 0.0;
    case SegmentType::Descent:
      if (s.descent_rate_mps > 0.0) return std::abs(s.altitude_change_m) / s.descent_rate_mps;
      return 0.0;
    case SegmentType::Transition:
    case SegmentType::Hover:
    case SegmentType::Reserve:
    default:
      return s.distance_m; // interpreted as duration
  }
}

inline void add_violation(double& viol, double amount) noexcept {
  if (!lift::bemt::is_finite(amount) || amount <= 0.0) return;
  if (!lift::bemt::is_finite(viol)) viol = 0.0;
  viol += amount;
}

inline MissionResult evaluate_mission(const MissionInputs& in_in, const ScoringConfig& cfg_in) {
  MissionInputs in = in_in;
  in.validate();

  ScoringConfig cfg = cfg_in;
  cfg.validate();

  MissionResult out;
  out.d_mass_kg = in.d_mass_kg;

  for (const auto& seg : in.segments) {
    SegmentResult r;
    r.type = seg.type;

    r.time_s = segment_time_s(seg);
    if (!lift::bemt::is_finite(r.time_s) || r.time_s < 0.0) r.time_s = 0.0;

    r.power_W = segment_power_W(seg, in);
    if (!lift::bemt::is_finite(r.power_W) || r.power_W < 0.0) r.power_W = 0.0;

    r.energy_J = r.power_W * r.time_s;
    if (!lift::bemt::is_finite(r.energy_J) || r.energy_J < 0.0) r.energy_J = 0.0;

    if (in.P_available_W > 0.0 && r.power_W > in.P_available_W) {
      r.power_exceeded = true;
      add_violation(out.violations, (r.power_W - in.P_available_W) / in.P_available_W);
    }

    out.total_time_s += r.time_s;
    out.total_energy_J += r.energy_J;
    out.segments.push_back(r);
  }

  if (!lift::bemt::is_finite(out.total_time_s) || out.total_time_s < 0.0) out.total_time_s = 0.0;
  if (!lift::bemt::is_finite(out.total_energy_J) || out.total_energy_J < 0.0) out.total_energy_J = 0.0;

  if (cfg.max_total_time_s > 0.0 && out.total_time_s > cfg.max_total_time_s) {
    add_violation(out.violations, (out.total_time_s - cfg.max_total_time_s) / cfg.max_total_time_s);
  }
  if (cfg.max_energy_J > 0.0 && out.total_energy_J > cfg.max_energy_J) {
    add_violation(out.violations, (out.total_energy_J - cfg.max_energy_J) / cfg.max_energy_J);
  }
  if (cfg.max_mass_kg > 0.0 && (in.d_mass_kg > 0.0) && (in.d_mass_kg > cfg.max_mass_kg)) {
    add_violation(out.violations, (in.d_mass_kg - cfg.max_mass_kg) / cfg.max_mass_kg);
  }
  if (cfg.max_power_W > 0.0 && in.P_available_W > cfg.max_power_W) {
    add_violation(out.violations, (in.P_available_W - cfg.max_power_W) / cfg.max_power_W);
  }

  out.score =
      cfg.w_time * out.total_time_s +
      cfg.w_energy * out.total_energy_J +
      cfg.w_mass * std::max(0.0, in.d_mass_kg) +
      cfg.w_viol * out.violations;

  if (!lift::bemt::is_finite(out.score) || out.score < 0.0) out.score = 0.0;

  out.code = lift::bemt::ErrorCode::Ok;
  return out;
}

} // namespace lift::mission
