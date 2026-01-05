/*
===============================================================================
Fragment 3.1.35 — SFCS Corridor Integration Screen (Routing, EMI/Isolation, Serviceability, Fault Isolation) (C++)
File: cpp/engine/integration/sfcs_corridor.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::integration {

// -----------------------------
// Enumerations
// -----------------------------
enum class NetType : std::uint8_t { Power = 0, Signal = 1, Comms = 2 };
enum class ShieldType : std::uint8_t { None = 0, Foil = 1, Braid = 2, SolidConduit = 3 };

enum class SfcsVerdict : std::uint8_t { Pass = 0, Fail = 1, Unknown = 2 };

struct SfcsCheck final {
  std::string clause_id;
  SfcsVerdict verdict = SfcsVerdict::Unknown;
  double value = 0.0;
  double threshold = 0.0;
  std::string message;
};

struct SfcsReport final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
  std::vector<SfcsCheck> checks;

  bool ok() const noexcept {
    if (code != lift::bemt::ErrorCode::Ok) return false;
    for (const auto& c : checks)
      if (c.verdict == SfcsVerdict::Fail) return false;
    return true;
  }
};

inline SfcsCheck make_check(std::string id, SfcsVerdict v, double val, double thr, std::string msg = {}) {
  return {std::move(id), v, val, thr, std::move(msg)};
}

inline SfcsCheck check_leq(const std::string& id, double val, double thr, const std::string& msg_fail) {
  if (!lift::bemt::is_finite(val) || !lift::bemt::is_finite(thr) || thr <= 0.0)
    return make_check(id, SfcsVerdict::Unknown, val, thr, "disabled/invalid");
  return make_check(id, (val <= thr ? SfcsVerdict::Pass : SfcsVerdict::Fail), val, thr, (val <= thr ? "" : msg_fail));
}

inline SfcsCheck check_geq(const std::string& id, double val, double thr, const std::string& msg_fail) {
  if (!lift::bemt::is_finite(val) || !lift::bemt::is_finite(thr) || thr <= 0.0)
    return make_check(id, SfcsVerdict::Unknown, val, thr, "disabled/invalid");
  return make_check(id, (val >= thr ? SfcsVerdict::Pass : SfcsVerdict::Fail), val, thr, (val >= thr ? "" : msg_fail));
}

// -----------------------------
// Corridor geometry & nets
// -----------------------------
struct CorridorSegment final {
  std::string id;                // stable id
  double length_m = 0.0;         // centerline length
  double area_m2 = 0.0;          // usable cross-sectional area for routing
  double min_bend_radius_m = 0.0; // geometric limit for embedded channel
  double access_points = 0.0;     // count of access points along segment (can be fractional weighting)
  bool removable_cover = false;   // serviceable cover present
};

struct NetSpec final {
  std::string id;         // stable id
  NetType type = NetType::Signal;
  ShieldType shield = ShieldType::None;

  // Electrical load proxies (used for EMI and thermal/I^2R screens)
  double I_A = 0.0;       // current (for power nets)
  double V_V = 0.0;       // nominal voltage (optional)
  double duty = 1.0;      // 0..1
  double req_separation_m = 0.0; // required minimum separation from power if this is sensitive (optional)

  // Routing requirements
  double required_area_m2 = 0.0; // corridor area needed (bundle width proxy)
  double required_bend_radius_m = 0.0;

  // Fault isolation
  bool requires_fuse = false;
  bool requires_dual_path = false;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "NetSpec.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(I_A) && I_A >= 0.0, lift::bemt::ErrorCode::InvalidInput, "NetSpec.I_A invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(V_V) && V_V >= 0.0, lift::bemt::ErrorCode::InvalidInput, "NetSpec.V_V invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(duty) && duty >= 0.0 && duty <= 1.0, lift::bemt::ErrorCode::InvalidInput, "NetSpec.duty invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(req_separation_m) && req_separation_m >= 0.0, lift::bemt::ErrorCode::InvalidInput, "NetSpec.req_separation invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(required_area_m2) && required_area_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "NetSpec.required_area invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(required_bend_radius_m) && required_bend_radius_m >= 0.0, lift::bemt::ErrorCode::InvalidInput, "NetSpec.required_bend_radius invalid");
  }
};

struct RouteAssignment final {
  std::string net_id;
  std::string seg_id;

  double proximity_to_power_m = -1.0; // <0 disables EMI proximity terms
  double parallel_run_m = 0.0;

  bool has_disconnect = false;
  bool has_fuse = false;
};

inline double shield_attenuation_factor(ShieldType s) noexcept {
  switch (s) {
    case ShieldType::Foil: return 0.5;
    case ShieldType::Braid: return 0.25;
    case ShieldType::SolidConduit: return 0.10;
    default: return 1.0;
  }
}

// -----------------------------
// Configuration thresholds
// -----------------------------
struct SfcsConfig final {
  // Routing
  double max_fill_frac = 0.70;          // sum(required_area)/area <= max_fill_frac
  double min_access_points_total = 1.0; // serviceability minimum across corridor
  double min_disconnect_frac = 0.25;    // fraction of nets with at least one disconnect
  double min_fuse_coverage_frac = 0.80; // fraction of fuse-required power nets with fusing

  // Isolation / EMI
  double min_power_signal_separation_m = 0.010; // 10 mm default
  double emi_risk_max = 1.0;                    // unitless proxy threshold

  // Electrical/thermal proxy for embedded conductors
  double max_I2R_loss_W = 0.0;      // <=0 disables
  double max_voltage_drop_V = 0.0;  // <=0 disables

  // Fault isolation
  double min_segmentation = 1.0;    // minimum number of independent zones (integer-ish)
  double max_spof_count = 0.0;      // <=0 disables

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_fill_frac) && max_fill_frac > 0.0 && max_fill_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.max_fill_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_access_points_total) && min_access_points_total >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.min_access_points_total invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_disconnect_frac) && min_disconnect_frac >= 0.0 && min_disconnect_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.min_disconnect_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_fuse_coverage_frac) && min_fuse_coverage_frac >= 0.0 && min_fuse_coverage_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.min_fuse_coverage_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_power_signal_separation_m) && min_power_signal_separation_m >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.min_power_signal_separation invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(emi_risk_max) && emi_risk_max > 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.emi_risk_max invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_I2R_loss_W) && max_I2R_loss_W >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.max_I2R_loss invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_voltage_drop_V) && max_voltage_drop_V >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.max_voltage_drop invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_segmentation) && min_segmentation >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.min_segmentation invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_spof_count) && max_spof_count >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "SfcsConfig.max_spof_count invalid");
  }
};

// Embedded conductor proxy parameters (for printed power channels)
struct ConductorModel final {
  double resistivity_ohm_m = 1.68e-8;       // Copper ~1.68e-8
  double default_conductor_area_m2 = 5e-6;  // ~5 mm^2

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(resistivity_ohm_m) && resistivity_ohm_m > 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "ConductorModel.resistivity invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(default_conductor_area_m2) && default_conductor_area_m2 > 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "ConductorModel.default_area invalid");
  }
};

// -----------------------------
// Outputs / metrics
// -----------------------------
struct SfcsMetrics final {
  double corridor_area_total_m2 = 0.0;
  double required_area_total_m2 = 0.0;
  double fill_fraction = 0.0;

  double total_length_m = 0.0;
  double access_points_total = 0.0;

  double disconnect_frac = 0.0;
  double emi_risk = 0.0;

  double I2R_loss_W = 0.0;
  double max_voltage_drop_V = 0.0;

  double segmentation_score = 0.0; // count of distinct segments used by nets
  double fuse_coverage_frac = 0.0;
  double spof_count = 0.0;         // number of dual-path-required nets lacking redundancy
};

// -----------------------------
// Utilities
// -----------------------------
inline const CorridorSegment* find_seg(const std::vector<CorridorSegment>& segs, const std::string& id) noexcept {
  for (const auto& s : segs)
    if (s.id == id) return &s;
  return nullptr;
}
inline const NetSpec* find_net(const std::vector<NetSpec>& nets, const std::string& id) noexcept {
  for (const auto& n : nets)
    if (n.id == id) return &n;
  return nullptr;
}

inline bool is_power(const NetSpec& n) noexcept { return n.type == NetType::Power; }

// Conservative EMI risk proxy:
inline double emi_risk_term(double I_A, double duty, double parallel_m, double proximity_m,
                            double sep_min_m, ShieldType shield) noexcept {
  if (!lift::bemt::is_finite(I_A) || !lift::bemt::is_finite(duty) || !lift::bemt::is_finite(parallel_m) || !lift::bemt::is_finite(proximity_m))
    return 0.0;
  if (I_A <= 0.0 || duty <= 0.0 || parallel_m <= 0.0) return 0.0;

  const double prox = std::max(proximity_m, sep_min_m);
  if (prox <= 0.0) return 0.0;

  const double sh = shield_attenuation_factor(shield);
  const double term = (I_A * duty) * (parallel_m / prox) * sh;
  return (lift::bemt::is_finite(term) && term >= 0.0) ? term : 0.0;
}

// Electrical loss for a net routed through a segment: R = rho * L / Acond
inline double conductor_R_ohm(double resistivity, double L_m, double Acond_m2) noexcept {
  if (!lift::bemt::is_finite(resistivity) || !lift::bemt::is_finite(L_m) || !lift::bemt::is_finite(Acond_m2)) return 0.0;
  if (resistivity <= 0.0 || L_m <= 0.0 || Acond_m2 <= 0.0) return 0.0;
  const double R = resistivity * (L_m / Acond_m2);
  return (lift::bemt::is_finite(R) && R >= 0.0) ? R : 0.0;
}

// -----------------------------
// Main evaluation
// -----------------------------
struct SfcsEvalOut final {
  SfcsMetrics metrics{};
  SfcsReport report{};
};

inline SfcsEvalOut evaluate_sfcs_corridor(const std::vector<CorridorSegment>& segments,
                                          const std::vector<NetSpec>& nets,
                                          const std::vector<RouteAssignment>& routes,
                                          const SfcsConfig& cfg_in,
                                          const ConductorModel& cond_in = {}) {
  SfcsConfig cfg = cfg_in;
  cfg.validate();

  ConductorModel cond = cond_in;
  cond.validate();

  SfcsEvalOut out;

  // Segment totals
  double area_total = 0.0;
  double length_total = 0.0;
  double access_total = 0.0;
  for (const auto& s : segments) {
    LIFT_BEMT_REQUIRE(!s.id.empty(), lift::bemt::ErrorCode::InvalidInput, "CorridorSegment.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(s.length_m) && s.length_m >= 0.0, lift::bemt::ErrorCode::InvalidInput, "CorridorSegment.length invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(s.area_m2) && s.area_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "CorridorSegment.area invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(s.min_bend_radius_m) && s.min_bend_radius_m >= 0.0, lift::bemt::ErrorCode::InvalidInput, "CorridorSegment.bend invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(s.access_points) && s.access_points >= 0.0, lift::bemt::ErrorCode::InvalidInput, "CorridorSegment.access invalid");

    area_total += s.area_m2;
    length_total += s.length_m;
    access_total += s.access_points + (s.removable_cover ? 0.25 : 0.0); // small deterministic bonus
  }

  out.metrics.corridor_area_total_m2 = lift::bemt::is_finite(area_total) ? area_total : 0.0;
  out.metrics.total_length_m = lift::bemt::is_finite(length_total) ? length_total : 0.0;
  out.metrics.access_points_total = lift::bemt::is_finite(access_total) ? access_total : 0.0;

  // Routing requirements
  double req_area = 0.0;
  std::size_t net_count = 0;
  std::size_t net_with_disconnect = 0;

  std::size_t fuse_required = 0;
  std::size_t fuse_covered = 0;

  std::size_t dual_required = 0;
  std::size_t dual_satisfied = 0;

  std::vector<std::string> used_seg_ids;

  auto count_routes_for_net = [&](const std::string& net_id) -> std::size_t {
    std::size_t c = 0;
    for (const auto& r : routes)
      if (r.net_id == net_id) ++c;
    return c;
  };

  double emi_risk = 0.0;
  double I2R_total_W = 0.0;
  double max_vdrop_V = 0.0;

  for (const auto& n : nets) {
    n.validate();
    ++net_count;
    req_area += n.required_area_m2;

    bool any_route = false;
    bool any_disc = false;
    bool any_fuse = false;
    double net_R_ohm = 0.0;
    double net_prox_m = -1.0;

    for (const auto& r : routes) {
      if (r.net_id != n.id) continue;
      any_route = true;

      const auto* seg = find_seg(segments, r.seg_id);
      LIFT_BEMT_REQUIRE(seg != nullptr, lift::bemt::ErrorCode::InvalidInput, "RouteAssignment references missing segment");

      used_seg_ids.push_back(seg->id);

      if (n.required_bend_radius_m > 0.0 && seg->min_bend_radius_m > 0.0) {
        if (n.required_bend_radius_m > seg->min_bend_radius_m) {
          out.report.checks.push_back(make_check(
              "SFCS.ROUTING.BEND_RADIUS",
              SfcsVerdict::Fail,
              n.required_bend_radius_m,
              seg->min_bend_radius_m,
              "net bend radius requirement exceeds segment capability"));
        }
      }

      any_disc = any_disc || r.has_disconnect;
      any_fuse = any_fuse || r.has_fuse;

      if (r.proximity_to_power_m >= 0.0 && r.parallel_run_m > 0.0) {
        net_prox_m = (net_prox_m < 0.0) ? r.proximity_to_power_m : std::min(net_prox_m, r.proximity_to_power_m);

        const double sep_min = std::max(cfg.min_power_signal_separation_m,
                                        (n.req_separation_m > 0.0 ? n.req_separation_m : 0.0));
        emi_risk += emi_risk_term(n.I_A, n.duty, r.parallel_run_m, r.proximity_to_power_m, sep_min, n.shield);
      }

      if (is_power(n) && n.I_A > 0.0 && seg->length_m > 0.0) {
        const double Acond = (n.required_area_m2 > 0.0) ? n.required_area_m2 : cond.default_conductor_area_m2;
        net_R_ohm += conductor_R_ohm(cond.resistivity_ohm_m, seg->length_m, Acond);
      }
    }

    if (!any_route) {
      out.report.checks.push_back(make_check(
          "SFCS.ROUTING.MISSING",
          SfcsVerdict::Fail,
          1.0,
          0.0,
          "net has no route assignment"));
    }

    if (any_disc) ++net_with_disconnect;

    if (n.requires_fuse) {
      ++fuse_required;
      if (any_fuse) ++fuse_covered;
    }

    if (n.requires_dual_path) {
      ++dual_required;
      if (count_routes_for_net(n.id) >= 2) ++dual_satisfied;
    }

    if (is_power(n) && n.I_A > 0.0) {
      const double Ieff = n.I_A * std::sqrt(std::max(0.0, n.duty));
      const double Ploss = Ieff * Ieff * net_R_ohm;
      if (lift::bemt::is_finite(Ploss) && Ploss >= 0.0) I2R_total_W += Ploss;

      if (n.V_V > 0.0) {
        const double Vdrop = Ieff * net_R_ohm;
        if (lift::bemt::is_finite(Vdrop) && Vdrop >= 0.0) max_vdrop_V = std::max(max_vdrop_V, Vdrop);
      }
    }

    if (net_prox_m >= 0.0) {
      const double sep_req = std::max(cfg.min_power_signal_separation_m,
                                      (n.req_separation_m > 0.0 ? n.req_separation_m : 0.0));
      out.report.checks.push_back(check_geq(
          "SFCS.ISOLATION.MIN_SEPARATION",
          net_prox_m,
          sep_req,
          "power/signal separation below requirement"));
    }
  }

  std::sort(used_seg_ids.begin(), used_seg_ids.end());
  used_seg_ids.erase(std::unique(used_seg_ids.begin(), used_seg_ids.end()), used_seg_ids.end());

  out.metrics.required_area_total_m2 = lift::bemt::is_finite(req_area) ? req_area : 0.0;
  out.metrics.fill_fraction = (out.metrics.corridor_area_total_m2 > 0.0)
      ? (out.metrics.required_area_total_m2 / out.metrics.corridor_area_total_m2)
      : 0.0;

  out.metrics.disconnect_frac = (net_count > 0) ? (static_cast<double>(net_with_disconnect) / static_cast<double>(net_count)) : 0.0;

  out.metrics.emi_risk = lift::bemt::is_finite(emi_risk) ? emi_risk : 0.0;

  out.metrics.I2R_loss_W = lift::bemt::is_finite(I2R_total_W) ? I2R_total_W : 0.0;
  out.metrics.max_voltage_drop_V = lift::bemt::is_finite(max_vdrop_V) ? max_vdrop_V : 0.0;

  out.metrics.segmentation_score = static_cast<double>(used_seg_ids.size());

  out.metrics.fuse_coverage_frac = (fuse_required > 0)
      ? (static_cast<double>(fuse_covered) / static_cast<double>(fuse_required))
      : 1.0; // if none required, treat as covered

  out.metrics.spof_count = (dual_required > 0)
      ? static_cast<double>(dual_required - dual_satisfied)
      : 0.0;

  // Core checks (GO/NO-GO style)
  out.report.checks.push_back(check_leq(
      "SFCS.ROUTING.FILL_FRAC_MAX",
      out.metrics.fill_fraction,
      cfg.max_fill_frac,
      "corridor fill fraction exceeds limit"));

  out.report.checks.push_back(check_geq(
      "SFCS.SVC.ACCESS_POINTS_MIN",
      out.metrics.access_points_total,
      cfg.min_access_points_total,
      "insufficient access points for serviceability"));

  out.report.checks.push_back(check_geq(
      "SFCS.SVC.DISCONNECT_FRAC_MIN",
      out.metrics.disconnect_frac,
      cfg.min_disconnect_frac,
      "insufficient modular disconnect coverage"));

  out.report.checks.push_back(check_leq(
      "SFCS.EMI.RISK_MAX",
      out.metrics.emi_risk,
      cfg.emi_risk_max,
      "EMI risk proxy exceeds max"));

  out.report.checks.push_back(check_geq(
      "SFCS.FAULT.SEGMENTATION_MIN",
      out.metrics.segmentation_score,
      cfg.min_segmentation,
      "insufficient corridor segmentation for fault isolation"));

  out.report.checks.push_back(check_geq(
      "SFCS.FAULT.FUSE_COVERAGE_MIN",
      out.metrics.fuse_coverage_frac,
      cfg.min_fuse_coverage_frac,
      "fuse coverage below minimum"));

  if (cfg.max_spof_count > 0.0) {
    out.report.checks.push_back(check_leq(
        "SFCS.FAULT.SPOF_MAX",
        out.metrics.spof_count,
        cfg.max_spof_count,
        "single-point-of-failure count too high (dual-path unmet)"));
  } else {
    out.report.checks.push_back(make_check("SFCS.FAULT.SPOF_MAX", SfcsVerdict::Unknown,
                                           out.metrics.spof_count, cfg.max_spof_count, "disabled"));
  }

  if (cfg.max_I2R_loss_W > 0.0) {
    out.report.checks.push_back(check_leq(
        "SFCS.ELEC.I2R_LOSS_MAX",
        out.metrics.I2R_loss_W,
        cfg.max_I2R_loss_W,
        "embedded conductor I^2R loss exceeds limit"));
  } else {
    out.report.checks.push_back(make_check("SFCS.ELEC.I2R_LOSS_MAX", SfcsVerdict::Unknown,
                                           out.metrics.I2R_loss_W, cfg.max_I2R_loss_W, "disabled"));
  }

  if (cfg.max_voltage_drop_V > 0.0) {
    out.report.checks.push_back(check_leq(
        "SFCS.ELEC.VDROP_MAX",
        out.metrics.max_voltage_drop_V,
        cfg.max_voltage_drop_V,
        "voltage drop exceeds limit"));
  } else {
    out.report.checks.push_back(make_check("SFCS.ELEC.VDROP_MAX", SfcsVerdict::Unknown,
                                           out.metrics.max_voltage_drop_V, cfg.max_voltage_drop_V, "disabled"));
  }

  out.report.code = lift::bemt::ErrorCode::Ok;
  return out;
}

} // namespace lift::integration
