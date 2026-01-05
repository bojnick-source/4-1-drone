/*
===============================================================================
Fragment 3.1.36 — Disk Area & Induced Power Closeout (A_total, DL, P_hover_1g, P_sized(k), Sensitivities) (C++)
File: cpp/engine/physics/disk_loading.hpp
===============================================================================
*/

#pragma once

#include "bemt_require.hpp"
#include "bemt_safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::physics {

struct DiskSpec final {
  std::string id;            // stable id
  double area_m2 = 0.0;      // geometric disk/inlet area
  std::string overlap_group; // same group => counts once (use for coax-in-duct)
  double weight = 1.0;       // optional weighting (default 1.0)

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "DiskSpec.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(area_m2) && area_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DiskSpec.area invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(weight) && weight >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DiskSpec.weight invalid");
  }
};

struct OverlapPair final {
  std::string a_id; // disk id or group key (caller-defined)
  std::string b_id; // disk id or group key
  double overlap_frac = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(!a_id.empty() && !b_id.empty(), lift::bemt::ErrorCode::InvalidInput, "OverlapPair ids empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(overlap_frac) && overlap_frac >= 0.0 && overlap_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidInput, "OverlapPair.overlap_frac invalid");
  }
};

struct Atmosphere final {
  double rho_kg_m3 = 1.225;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(rho_kg_m3) && rho_kg_m3 > 0.0 && rho_kg_m3 < 5.0,
                      lift::bemt::ErrorCode::InvalidInput, "Atmosphere.rho invalid");
  }
};

struct DiskLoadingConfig final {
  double FM_default = 0.75;
  double P_additional_W = 0.0;
  double sizing_k = 1.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(FM_default) && FM_default > 0.0 && FM_default <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "DiskLoadingConfig.FM_default invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_additional_W) && P_additional_W >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "DiskLoadingConfig.P_additional invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(sizing_k) && sizing_k >= 1.0 && sizing_k <= 5.0,
                      lift::bemt::ErrorCode::InvalidConfig, "DiskLoadingConfig.sizing_k invalid");
  }
};

struct DiskLoadingMetrics final {
  double A_total_m2 = 0.0;
  double DL_N_m2 = 0.0;

  double P_induced_ideal_W = 0.0;
  double P_induced_actual_W = 0.0;
  double P_hover_total_W = 0.0;
  double P_sized_W = 0.0;

  double dP_dA_W_per_m2 = 0.0;
  double dP_dT_W_per_N = 0.0;

  double dlnP_dlnA = 0.0;
  double dlnP_dlnT = 0.0;
};

inline std::string overlap_key(const DiskSpec& d) {
  if (!d.overlap_group.empty()) return std::string("G:") + d.overlap_group;
  return std::string("D:") + d.id;
}

inline double compute_effective_area(const std::vector<DiskSpec>& disks,
                                     const std::vector<OverlapPair>& overlaps = {}) {
  struct KeyArea {
    std::string key;
    double area = 0.0;
  };

  std::vector<KeyArea> agg;
  agg.reserve(disks.size());

  for (const auto& d : disks) {
    d.validate();
    const double A = std::max(0.0, d.area_m2 * d.weight);
    const std::string k = overlap_key(d);

    auto it = std::find_if(agg.begin(), agg.end(), [&](const KeyArea& ka) { return ka.key == k; });
    if (it == agg.end()) agg.push_back({k, A});
    else it->area = std::max(it->area, A);
  }

  double A_total = 0.0;
  for (const auto& ka : agg) A_total += ka.area;

  if (!lift::bemt::is_finite(A_total) || A_total < 0.0) A_total = 0.0;

  if (!overlaps.empty() && !agg.empty()) {
    auto find_area = [&](const std::string& key) -> double {
      for (const auto& ka : agg)
        if (ka.key == key) return ka.area;
      for (const auto& ka : agg)
        if (ka.key == ("D:" + key) || ka.key == ("G:" + key)) return ka.area;
      return 0.0;
    };

    double overlap_sub = 0.0;
    for (const auto& p : overlaps) {
      p.validate();
      const double Aa = find_area(p.a_id);
      const double Ab = find_area(p.b_id);
      if (Aa <= 0.0 || Ab <= 0.0) continue;
      overlap_sub += p.overlap_frac * std::min(Aa, Ab);
    }

    if (lift::bemt::is_finite(overlap_sub) && overlap_sub > 0.0) {
      A_total = std::max(0.0, A_total - overlap_sub);
    }
  }

  return A_total;
}

inline double induced_power_ideal(double thrust_N, double rho, double A_m2) noexcept {
  if (!lift::bemt::is_finite(thrust_N) || !lift::bemt::is_finite(rho) || !lift::bemt::is_finite(A_m2)) return 0.0;
  if (thrust_N <= 0.0 || rho <= 0.0 || A_m2 <= 0.0) return 0.0;
  const double denom = lift::bemt::safe_sqrt(2.0 * rho * A_m2, 0.0);
  if (denom <= 0.0) return 0.0;
  const double T32 = thrust_N * lift::bemt::safe_sqrt(thrust_N, 0.0);
  const double P = T32 / denom;
  return (lift::bemt::is_finite(P) && P >= 0.0) ? P : 0.0;
}

inline DiskLoadingMetrics compute_disk_loading(const std::vector<DiskSpec>& disks,
                                               double thrust_required_N,
                                               const Atmosphere& atm_in,
                                               const DiskLoadingConfig& cfg_in,
                                               const std::vector<OverlapPair>& overlaps = {},
                                               double FM = 0.0,
                                               double eta_induced = 0.0) {
  Atmosphere atm = atm_in;
  atm.validate();

  DiskLoadingConfig cfg = cfg_in;
  cfg.validate();

  LIFT_BEMT_REQUIRE(lift::bemt::is_finite(thrust_required_N) && thrust_required_N >= 0.0,
                    lift::bemt::ErrorCode::InvalidInput, "thrust_required_N invalid");

  DiskLoadingMetrics m{};

  m.A_total_m2 = compute_effective_area(disks, overlaps);

  if (m.A_total_m2 > 0.0 && thrust_required_N > 0.0) {
    m.DL_N_m2 = thrust_required_N / m.A_total_m2;
    if (!lift::bemt::is_finite(m.DL_N_m2) || m.DL_N_m2 < 0.0) m.DL_N_m2 = 0.0;
  }

  m.P_induced_ideal_W = induced_power_ideal(thrust_required_N, atm.rho_kg_m3, m.A_total_m2);

  double eff = 0.0;
  if (lift::bemt::is_finite(FM) && FM > 0.0 && FM <= 1.0) eff = FM;
  else if (lift::bemt::is_finite(eta_induced) && eta_induced > 0.0 && eta_induced <= 1.0) eff = eta_induced;
  else eff = cfg.FM_default;

  if (!lift::bemt::is_finite(eff) || eff <= 0.0) eff = cfg.FM_default;

  m.P_induced_actual_W = (eff > 0.0) ? (m.P_induced_ideal_W / eff) : 0.0;
  if (!lift::bemt::is_finite(m.P_induced_actual_W) || m.P_induced_actual_W < 0.0) m.P_induced_actual_W = 0.0;

  m.P_hover_total_W = m.P_induced_actual_W + cfg.P_additional_W;
  if (!lift::bemt::is_finite(m.P_hover_total_W) || m.P_hover_total_W < 0.0) m.P_hover_total_W = 0.0;

  m.P_sized_W = cfg.sizing_k * m.P_hover_total_W;
  if (!lift::bemt::is_finite(m.P_sized_W) || m.P_sized_W < 0.0) m.P_sized_W = 0.0;

  if (m.P_induced_ideal_W > 0.0 && m.A_total_m2 > 0.0) {
    m.dP_dA_W_per_m2 = -0.5 * (m.P_induced_ideal_W / m.A_total_m2);
    if (!lift::bemt::is_finite(m.dP_dA_W_per_m2)) m.dP_dA_W_per_m2 = 0.0;
    m.dlnP_dlnA = -0.5;
  }
  if (m.P_induced_ideal_W > 0.0 && thrust_required_N > 0.0) {
    m.dP_dT_W_per_N = 1.5 * (m.P_induced_ideal_W / thrust_required_N);
    if (!lift::bemt::is_finite(m.dP_dT_W_per_N)) m.dP_dT_W_per_N = 0.0;
    m.dlnP_dlnT = 1.5;
  }

  return m;
}

} // namespace lift::physics
