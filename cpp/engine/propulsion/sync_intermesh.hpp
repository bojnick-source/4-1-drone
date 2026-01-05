/*
===============================================================================
Fragment 3.1.39 — Sync/Intermeshing Feasibility (Phase Tolerance, Disturbances, Latency, Fault Tree) (C++)
File: cpp/engine/propulsion/sync_intermesh.hpp
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

namespace lift::propulsion {

enum class SyncVerdict : std::uint8_t { Pass = 0, Fail = 1, Unknown = 2 };
enum class FaultSeverity : std::uint8_t { Minor = 0, Major = 1, Catastrophic = 2 };

inline double clamp_nonneg(double x) noexcept {
  if (!lift::bemt::is_finite(x) || x < 0.0) return 0.0;
  return x;
}

inline double deg2rad(double deg) noexcept { return deg * 0.017453292519943295; }

struct RotorPairGeometry final {
  std::string id;
  bool intermeshing = false;
  bool coaxial_same_footprint = false;
  std::uint32_t blades_a = 2;
  std::uint32_t blades_b = 2;
  double rpm_a = 0.0;
  double rpm_b = 0.0;
  double clearance_deg = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "RotorPairGeometry.id empty");
    LIFT_BEMT_REQUIRE(blades_a >= 1 && blades_b >= 1, lift::bemt::ErrorCode::InvalidInput, "blade counts invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(rpm_a) && rpm_a >= 0.0, lift::bemt::ErrorCode::InvalidInput, "rpm_a invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(rpm_b) && rpm_b >= 0.0, lift::bemt::ErrorCode::InvalidInput, "rpm_b invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(clearance_deg) && clearance_deg >= 0.0 && clearance_deg <= 180.0,
                      lift::bemt::ErrorCode::InvalidInput, "clearance_deg invalid");
  }

  double omega_a_rad_s() const noexcept { return (rpm_a * 2.0 * M_PI) / 60.0; }
  double omega_b_rad_s() const noexcept { return (rpm_b * 2.0 * M_PI) / 60.0; }
  double clearance_rad() const noexcept { return deg2rad(clearance_deg); }
};

struct SyncConfig final {
  double clearance_margin = 1.50;
  double latency_s = 0.0;
  double phase_sensor_err_deg = 0.0;
  double phase_correction_max_deg = 0.0;
  double control_rate_hz = 0.0;
  double torque_ripple_frac = 0.0;
  double aero_phase_disturb_deg = 0.0;
  double model_uncertainty_deg = 0.0;
  bool catastrophic_on_fail = true;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(clearance_margin) && clearance_margin >= 1.0 && clearance_margin <= 5.0,
                      lift::bemt::ErrorCode::InvalidConfig, "clearance_margin invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(latency_s) && latency_s >= 0.0 && latency_s <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "latency_s invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(phase_sensor_err_deg) && phase_sensor_err_deg >= 0.0 && phase_sensor_err_deg <= 30.0,
                      lift::bemt::ErrorCode::InvalidConfig, "phase_sensor_err invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(phase_correction_max_deg) && phase_correction_max_deg >= 0.0 && phase_correction_max_deg <= 90.0,
                      lift::bemt::ErrorCode::InvalidConfig, "phase_correction_max invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(control_rate_hz) && control_rate_hz >= 0.0 && control_rate_hz <= 2000.0,
                      lift::bemt::ErrorCode::InvalidConfig, "control_rate_hz invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(torque_ripple_frac) && torque_ripple_frac >= 0.0 && torque_ripple_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "torque_ripple_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(aero_phase_disturb_deg) && aero_phase_disturb_deg >= 0.0 && aero_phase_disturb_deg <= 90.0,
                      lift::bemt::ErrorCode::InvalidConfig, "aero_phase_disturb invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(model_uncertainty_deg) && model_uncertainty_deg >= 0.0 && model_uncertainty_deg <= 90.0,
                      lift::bemt::ErrorCode::InvalidConfig, "model_uncertainty invalid");
  }
};

struct FaultEvent final {
  std::string id;
  std::string description;
  FaultSeverity severity = FaultSeverity::Major;
  double probability = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "FaultEvent.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(probability) && probability >= 0.0 && probability <= 1.0,
                      lift::bemt::ErrorCode::InvalidInput, "FaultEvent.probability invalid");
  }
};

struct FaultGate final {
  enum class Type : std::uint8_t { And = 0, Or = 1 } type = Type::Or;
  std::string id;
  std::string description;
  std::vector<std::string> children;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "FaultGate.id empty");
    LIFT_BEMT_REQUIRE(!children.empty(), lift::bemt::ErrorCode::InvalidInput, "FaultGate.children empty");
  }
};

struct FaultTree final {
  std::vector<FaultEvent> events;
  std::vector<FaultGate> gates;
  std::string top_gate_id;

  void validate() const {
    LIFT_BEMT_REQUIRE(!top_gate_id.empty(), lift::bemt::ErrorCode::InvalidInput, "FaultTree.top_gate_id empty");
    for (const auto& e : events) e.validate();
    for (const auto& g : gates) g.validate();
  }
};

struct SyncMetrics final {
  double clearance_rad = 0.0;
  double allowable_rad = 0.0;
  double phi_static_rad = 0.0;
  double phi_sensor_rad = 0.0;
  double phi_latency_rad = 0.0;
  double phi_disturb_rad = 0.0;
  double phi_actuator_rad = 0.0;
  double phi_total_rad = 0.0;
  double margin = 0.0;
};

struct SyncCheck final {
  std::string id;
  SyncVerdict verdict = SyncVerdict::Unknown;
  double value = 0.0;
  double threshold = 0.0;
  std::string message;
};

struct SyncReport final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
  std::vector<SyncCheck> checks;

  bool ok() const noexcept {
    if (code != lift::bemt::ErrorCode::Ok) return false;
    for (const auto& c : checks)
      if (c.verdict == SyncVerdict::Fail) return false;
    return true;
  }
};

struct SyncEvalOut final {
  SyncMetrics metrics{};
  SyncReport report{};
  bool fault_tree_present = false;
  double top_event_prob = 0.0;
};

inline double latency_phase_error_rad(double omega_rel, double latency_s) noexcept {
  const double val = std::abs(omega_rel) * clamp_nonneg(latency_s);
  return (lift::bemt::is_finite(val) && val >= 0.0) ? val : 0.0;
}

inline double sensor_error_bound_rad(double sensor_err_deg) noexcept {
  const double b = 3.0 * deg2rad(clamp_nonneg(sensor_err_deg));
  return (lift::bemt::is_finite(b) && b >= 0.0) ? b : 0.0;
}

inline double model_error_bound_rad(double model_uncert_deg) noexcept {
  const double b = deg2rad(clamp_nonneg(model_uncert_deg));
  return (lift::bemt::is_finite(b) && b >= 0.0) ? b : 0.0;
}

inline double disturbance_error_bound_rad(double aero_deg, double torque_ripple_frac, double clearance_rad) noexcept {
  const double phi_aero = deg2rad(clamp_nonneg(aero_deg));
  const double frac = std::min(1.0, std::max(0.0, torque_ripple_frac));
  const double phi_ripple = frac * 0.25 * clamp_nonneg(clearance_rad);
  const double phi = phi_aero + phi_ripple;
  return (lift::bemt::is_finite(phi) && phi >= 0.0) ? phi : 0.0;
}

inline double actuator_residual_bound_rad(double correction_max_deg, double control_rate_hz) noexcept {
  if (!lift::bemt::is_finite(control_rate_hz) || control_rate_hz <= 0.0) return 0.0;
  const double corr = deg2rad(clamp_nonneg(correction_max_deg));
  const double phi = 0.5 * corr;
  return (lift::bemt::is_finite(phi) && phi >= 0.0) ? phi : 0.0;
}

inline const FaultEvent* find_event(const FaultTree& ft, const std::string& id) noexcept {
  for (const auto& e : ft.events)
    if (e.id == id) return &e;
  return nullptr;
}
inline const FaultGate* find_gate(const FaultTree& ft, const std::string& id) noexcept {
  for (const auto& g : ft.gates)
    if (g.id == id) return &g;
  return nullptr;
}

inline double eval_node_prob(const FaultTree& ft, const std::string& id, std::uint32_t depth = 0) {
  if (depth > 64) return 0.0;

  if (const auto* e = find_event(ft, id)) return e->probability;

  const auto* g = find_gate(ft, id);
  if (!g) return 0.0;

  if (g->type == FaultGate::Type::Or) {
    double prod = 1.0;
    for (const auto& c : g->children) {
      const double p = std::clamp(eval_node_prob(ft, c, depth + 1), 0.0, 1.0);
      prod *= (1.0 - p);
    }
    return std::clamp(1.0 - prod, 0.0, 1.0);
  } else {
    double prod = 1.0;
    for (const auto& c : g->children) {
      const double p = std::clamp(eval_node_prob(ft, c, depth + 1), 0.0, 1.0);
      prod *= p;
    }
    return std::clamp(prod, 0.0, 1.0);
  }
}

inline SyncEvalOut evaluate_sync_feasibility(const RotorPairGeometry& pair_in,
                                             const SyncConfig& cfg_in,
                                             const FaultTree* fault_tree = nullptr) {
  pair_in.validate();

  SyncConfig cfg = cfg_in;
  cfg.validate();

  SyncEvalOut out;

  const double clearance = pair_in.clearance_rad();
  out.metrics.clearance_rad = clearance;
  out.metrics.allowable_rad = (cfg.clearance_margin > 0.0) ? (clearance / cfg.clearance_margin) : 0.0;

  const double omega_a = pair_in.omega_a_rad_s();
  const double omega_b = pair_in.omega_b_rad_s();
  const double omega_rel = omega_a - omega_b;

  out.metrics.phi_static_rad = model_error_bound_rad(cfg.model_uncertainty_deg);
  out.metrics.phi_sensor_rad = sensor_error_bound_rad(cfg.phase_sensor_err_deg);
  out.metrics.phi_latency_rad = latency_phase_error_rad(omega_rel, cfg.latency_s);
  out.metrics.phi_disturb_rad = disturbance_error_bound_rad(cfg.aero_phase_disturb_deg, cfg.torque_ripple_frac, clearance);
  out.metrics.phi_actuator_rad = actuator_residual_bound_rad(cfg.phase_correction_max_deg, cfg.control_rate_hz);

  out.metrics.phi_total_rad =
      out.metrics.phi_static_rad +
      out.metrics.phi_sensor_rad +
      out.metrics.phi_latency_rad +
      out.metrics.phi_disturb_rad +
      out.metrics.phi_actuator_rad;

  if (!lift::bemt::is_finite(out.metrics.phi_total_rad) || out.metrics.phi_total_rad < 0.0)
    out.metrics.phi_total_rad = 0.0;

  if (out.metrics.phi_total_rad > 0.0) out.metrics.margin = out.metrics.allowable_rad / out.metrics.phi_total_rad;
  else out.metrics.margin = (out.metrics.allowable_rad > 0.0 ? 1e9 : 0.0);

  if (!lift::bemt::is_finite(out.metrics.margin) || out.metrics.margin < 0.0) out.metrics.margin = 0.0;

  auto push = [&](std::string id, SyncVerdict v, double val, double thr, std::string msg = {}) {
    out.report.checks.push_back({std::move(id), v, val, thr, std::move(msg)});
  };

  if (pair_in.intermeshing) {
    if (clearance <= 0.0) push("SYNC.CLEARANCE.POSITIVE", SyncVerdict::Fail, clearance, 0.0, "intermeshing requires positive clearance");
    else push("SYNC.CLEARANCE.POSITIVE", SyncVerdict::Pass, clearance, 0.0, "");
  } else {
    push("SYNC.CLEARANCE.POSITIVE", SyncVerdict::Unknown, clearance, 0.0, "not intermeshing");
  }

  if (out.metrics.allowable_rad <= 0.0) {
    push("SYNC.PHASE_BUDGET", SyncVerdict::Unknown, out.metrics.phi_total_rad, out.metrics.allowable_rad, "allowable budget invalid/zero");
  } else {
    const bool pass = out.metrics.phi_total_rad <= out.metrics.allowable_rad;
    push("SYNC.PHASE_BUDGET",
         pass ? SyncVerdict::Pass : SyncVerdict::Fail,
         out.metrics.phi_total_rad,
         out.metrics.allowable_rad,
         pass ? "" : "phase error budget exceeds allowable (strike risk)");
  }

  if (pair_in.intermeshing) {
    const double phi_lat = out.metrics.phi_latency_rad;
    const double thr = out.metrics.allowable_rad * 0.25;
    if (thr > 0.0) {
      const bool pass = phi_lat <= thr;
      push("SYNC.LATENCY_SHARE_MAX",
           pass ? SyncVerdict::Pass : SyncVerdict::Fail,
           phi_lat,
           thr,
           pass ? "" : "latency-induced drift consumes too much of phase budget");
    } else {
      push("SYNC.LATENCY_SHARE_MAX", SyncVerdict::Unknown, phi_lat, thr, "budget invalid");
    }
  } else {
    push("SYNC.LATENCY_SHARE_MAX", SyncVerdict::Unknown, out.metrics.phi_latency_rad, 0.0, "not intermeshing");
  }

  if (pair_in.intermeshing && cfg.control_rate_hz <= 0.0) {
    push("SYNC.CONTROL_RATE.NONZERO", SyncVerdict::Fail, cfg.control_rate_hz, 1.0, "intermeshing requires active sync control");
  } else if (pair_in.intermeshing) {
    push("SYNC.CONTROL_RATE.NONZERO", SyncVerdict::Pass, cfg.control_rate_hz, 1.0, "");
  } else {
    push("SYNC.CONTROL_RATE.NONZERO", SyncVerdict::Unknown, cfg.control_rate_hz, 1.0, "not intermeshing");
  }

  if (fault_tree) {
    fault_tree->validate();
    out.fault_tree_present = true;
    out.top_event_prob = eval_node_prob(*fault_tree, fault_tree->top_gate_id);

    if (pair_in.intermeshing && cfg.catastrophic_on_fail && out.top_event_prob > 0.0) {
      const double pmax = 1e-3;
      const bool pass = out.top_event_prob <= pmax;
      push("SYNC.FAULTTREE.TOP_PROB_MAX",
           pass ? SyncVerdict::Pass : SyncVerdict::Fail,
           out.top_event_prob,
           pmax,
           pass ? "" : "fault tree top event probability exceeds max (placeholder threshold)");
    } else if (pair_in.intermeshing && cfg.catastrophic_on_fail) {
      push("SYNC.FAULTTREE.TOP_PROB_MAX", SyncVerdict::Unknown, out.top_event_prob, 0.0, "unquantified probability");
    } else {
      push("SYNC.FAULTTREE.TOP_PROB_MAX", SyncVerdict::Unknown, out.top_event_prob, 0.0, "not enforced");
    }
  } else {
    out.fault_tree_present = false;
    out.top_event_prob = 0.0;
  }

  out.report.code = lift::bemt::ErrorCode::Ok;
  return out;
}

} // namespace lift::propulsion
