/*
===============================================================================
Fragment 3.1.33 — Numerical GO/NO-GO Threshold Registry (Stable Gate IDs + Deterministic Evaluation) (C++)
File: cpp/engine/closeout/gates.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace lift::closeout {

enum class GateCmp : std::uint8_t {
  LessEq = 0,
  GreaterEq = 1,
  RangeInclusive = 2
};

enum class GateSeverity : std::uint8_t {
  FailSoft = 0, // allowed to proceed but penalize / flag
  FailHard = 1  // hard NO-GO (candidate rejected)
};

enum class GateStatus : std::uint8_t {
  Pass = 0,
  Fail = 1,
  Unknown = 2
};

struct GateSpec final {
  std::string gate_id;     // stable id, e.g., "GATE.MASS.EMPTY_MAX"
  std::string metric_key;  // stable metric key, e.g., "mass.empty_kg"
  std::string clause_id;   // optional (for rules mapping), can match gate_id
  GateCmp cmp = GateCmp::LessEq;
  double a = 0.0;          // threshold or min
  double b = 0.0;          // max (range), else unused
  GateSeverity severity = GateSeverity::FailHard;

  // Optional note (not used by logic)
  std::string note;

  void validate() const {
    LIFT_BEMT_REQUIRE(!gate_id.empty(), lift::bemt::ErrorCode::InvalidConfig, "GateSpec.gate_id empty");
    LIFT_BEMT_REQUIRE(!metric_key.empty(), lift::bemt::ErrorCode::InvalidConfig, "GateSpec.metric_key empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(a), lift::bemt::ErrorCode::InvalidConfig, "GateSpec.a invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(b), lift::bemt::ErrorCode::InvalidConfig, "GateSpec.b invalid");
    if (cmp == GateCmp::RangeInclusive) {
      LIFT_BEMT_REQUIRE(a <= b, lift::bemt::ErrorCode::InvalidConfig, "GateSpec range a>b");
    }
  }
};

struct GateEval final {
  std::string gate_id;
  std::string metric_key;
  std::string clause_id;

  GateStatus status = GateStatus::Unknown;
  GateSeverity severity = GateSeverity::FailHard;

  double value = 0.0;
  double a = 0.0;
  double b = 0.0;

  std::string message;
};

struct GateReport final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
  std::vector<GateEval> evals;

  bool hard_pass() const noexcept {
    if (code != lift::bemt::ErrorCode::Ok) return false;
    for (const auto& e : evals) {
      if (e.status == GateStatus::Fail && e.severity == GateSeverity::FailHard) return false;
    }
    return true;
  }

  bool any_fail() const noexcept {
    for (const auto& e : evals)
      if (e.status == GateStatus::Fail) return true;
    return false;
  }
};

// Metric store: stable key -> numeric value.
// Caller populates this from hover_metrics, mass_ledger, parasite_drag, etc.
using MetricMap = std::unordered_map<std::string, double>;

inline const double* find_metric(const MetricMap& m, const std::string& key) noexcept {
  const auto it = m.find(key);
  return (it == m.end()) ? nullptr : &it->second;
}

inline GateEval eval_one(const GateSpec& g, const MetricMap& metrics) {
  g.validate();

  GateEval out;
  out.gate_id = g.gate_id;
  out.metric_key = g.metric_key;
  out.clause_id = g.clause_id.empty() ? g.gate_id : g.clause_id;
  out.severity = g.severity;
  out.a = g.a;
  out.b = g.b;

  const double* pv = find_metric(metrics, g.metric_key);
  if (!pv || !lift::bemt::is_finite(*pv)) {
    out.status = GateStatus::Unknown;
    out.message = "metric missing/invalid";
    out.value = 0.0;
    return out;
  }

  out.value = *pv;

  switch (g.cmp) {
    case GateCmp::LessEq: {
      out.status = (out.value <= g.a) ? GateStatus::Pass : GateStatus::Fail;
      out.message = (out.status == GateStatus::Pass) ? "" : "value exceeds max";
    } break;
    case GateCmp::GreaterEq: {
      out.status = (out.value >= g.a) ? GateStatus::Pass : GateStatus::Fail;
      out.message = (out.status == GateStatus::Pass) ? "" : "value below min";
    } break;
    case GateCmp::RangeInclusive: {
      out.status = (out.value >= g.a && out.value <= g.b) ? GateStatus::Pass : GateStatus::Fail;
      out.message = (out.status == GateStatus::Pass) ? "" : "value out of range";
    } break;
    default: {
      out.status = GateStatus::Unknown;
      out.message = "unknown comparator";
    } break;
  }

  return out;
}

inline GateReport eval_all(const std::vector<GateSpec>& gates, const MetricMap& metrics) {
  GateReport rep;
  rep.evals.reserve(gates.size());

  for (const auto& g : gates) {
    rep.evals.push_back(eval_one(g, metrics));
  }

  rep.code = lift::bemt::ErrorCode::Ok;
  return rep;
}

// Default gate set placeholders (fill real X/Y thresholds from primary rules + internal targets)
inline std::vector<GateSpec> default_gates_placeholders() {
  std::vector<GateSpec> g;

  // Example: empty mass cap (kg)
  g.push_back({
      "GATE.MASS.EMPTY_MAX",
      "mass.empty_kg",
      "GATE.MASS.EMPTY_MAX",
      GateCmp::LessEq,
      24.95,
      0.0,
      GateSeverity::FailHard,
      "Empty mass must be <= cap (replace if rule differs)"});

  // Example: payload minimum (kg)
  g.push_back({
      "GATE.PAYLOAD.MIN",
      "mass.payload_kg",
      "GATE.PAYLOAD.MIN",
      GateCmp::GreaterEq,
      99.8,
      0.0,
      GateSeverity::FailHard,
      "Payload must be >= minimum (replace with official value)"});

  // Example: total effective disk area minimum (m^2) — internal feasibility gate
  g.push_back({
      "GATE.DISK_AREA.MIN",
      "hover.A_total_m2",
      "GATE.DISK_AREA.MIN",
      GateCmp::GreaterEq,
      0.0,
      0.0,
      GateSeverity::FailSoft,
      "Internal gate; set based on feasibility screening"});

  // Example: disk loading maximum (N/m^2) — internal feasibility gate
  g.push_back({
      "GATE.DISK_LOADING.MAX",
      "hover.DL_N_m2",
      "GATE.DISK_LOADING.MAX",
      GateCmp::LessEq,
      0.0,
      0.0,
      GateSeverity::FailSoft,
      "Internal gate; set based on feasibility screening"});

  // Example: hover sized power maximum (W) — internal gate
  g.push_back({
      "GATE.HOVER_POWER.MAX",
      "hover.P_sized_W",
      "GATE.HOVER_POWER.MAX",
      GateCmp::LessEq,
      0.0,
      0.0,
      GateSeverity::FailSoft,
      "Internal gate; set based on propulsion limits"});

  return g;
}

} // namespace lift::closeout
