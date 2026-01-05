/*
===============================================================================
Fragment 3.1.37 — Parasite Drag & Cruise Delta Closeout (CdS, P_parasite(V), “Boom Removal” Check) (C++)
File: cpp/engine/physics/parasite_drag.hpp
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

struct DragItem final {
  std::string id;       // stable id (e.g., "boom.L1", "fuselage", "gearbox_housing")
  std::string group;    // optional grouping (e.g., "booms", "body", "landing_gear")
  double Cd = 0.0;      // dimensionless drag coefficient (optional if CdS provided)
  double S_ref_m2 = 0.0; // reference/frontal/wetted proxy area (caller-defined)
  double CdS_m2 = 0.0;  // if >0 overrides Cd*S_ref

  double interference = 1.0; // >=0

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "DragItem.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(Cd) && Cd >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DragItem.Cd invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(S_ref_m2) && S_ref_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DragItem.S_ref invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(CdS_m2) && CdS_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DragItem.CdS invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(interference) && interference >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DragItem.interference invalid");
  }

  double effective_CdS() const noexcept {
    const double base = (CdS_m2 > 0.0) ? CdS_m2 : (Cd * S_ref_m2);
    const double eff = base * interference;
    return (lift::bemt::is_finite(eff) && eff >= 0.0) ? eff : 0.0;
  }
};

struct DragTotals final {
  double CdS_total_m2 = 0.0;
};

struct Atmosphere final {
  double rho_kg_m3 = 1.225;
  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(rho_kg_m3) && rho_kg_m3 > 0.0 && rho_kg_m3 < 5.0,
                      lift::bemt::ErrorCode::InvalidInput, "Atmosphere.rho invalid");
  }
};

inline DragTotals compute_drag_totals(const std::vector<DragItem>& items) {
  DragTotals t{};
  double sum = 0.0;
  for (const auto& it : items) {
    it.validate();
    sum += it.effective_CdS();
  }
  if (!lift::bemt::is_finite(sum) || sum < 0.0) sum = 0.0;
  t.CdS_total_m2 = sum;
  return t;
}

inline double parasite_drag_N(double rho, double V_mps, double CdS_m2) noexcept {
  if (!lift::bemt::is_finite(rho) || !lift::bemt::is_finite(V_mps) || !lift::bemt::is_finite(CdS_m2)) return 0.0;
  if (rho <= 0.0 || V_mps <= 0.0 || CdS_m2 <= 0.0) return 0.0;
  const double D = 0.5 * rho * V_mps * V_mps * CdS_m2;
  return (lift::bemt::is_finite(D) && D >= 0.0) ? D : 0.0;
}

inline double parasite_power_W(double rho, double V_mps, double CdS_m2) noexcept {
  const double D = parasite_drag_N(rho, V_mps, CdS_m2);
  const double P = D * V_mps;
  return (lift::bemt::is_finite(P) && P >= 0.0) ? P : 0.0;
}

struct DragDelta final {
  DragTotals base{};
  DragTotals cand{};

  double d_CdS_m2 = 0.0;

  double V_target_mps = 0.0;
  double D_base_N = 0.0;
  double D_cand_N = 0.0;
  double d_D_N = 0.0;

  double P_base_W = 0.0;
  double P_cand_W = 0.0;
  double d_P_W = 0.0;
};

inline DragDelta compare_drag(const std::vector<DragItem>& baseline,
                              const std::vector<DragItem>& candidate,
                              const Atmosphere& atm_in,
                              double V_target_mps = 0.0) {
  Atmosphere atm = atm_in;
  atm.validate();

  DragDelta d{};
  d.base = compute_drag_totals(baseline);
  d.cand = compute_drag_totals(candidate);

  d.d_CdS_m2 = d.cand.CdS_total_m2 - d.base.CdS_total_m2;
  if (!lift::bemt::is_finite(d.d_CdS_m2)) d.d_CdS_m2 = 0.0;

  d.V_target_mps = (lift::bemt::is_finite(V_target_mps) && V_target_mps > 0.0) ? V_target_mps : 0.0;
  if (d.V_target_mps > 0.0) {
    d.D_base_N = parasite_drag_N(atm.rho_kg_m3, d.V_target_mps, d.base.CdS_total_m2);
    d.D_cand_N = parasite_drag_N(atm.rho_kg_m3, d.V_target_mps, d.cand.CdS_total_m2);
    d.d_D_N = d.D_cand_N - d.D_base_N;

    d.P_base_W = parasite_power_W(atm.rho_kg_m3, d.V_target_mps, d.base.CdS_total_m2);
    d.P_cand_W = parasite_power_W(atm.rho_kg_m3, d.V_target_mps, d.cand.CdS_total_m2);
    d.d_P_W = d.P_cand_W - d.P_base_W;

    if (!lift::bemt::is_finite(d.d_D_N)) d.d_D_N = 0.0;
    if (!lift::bemt::is_finite(d.d_P_W)) d.d_P_W = 0.0;
  }

  return d;
}

struct BoomRemovalCheckOut final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;

  double CdS_booms_base_m2 = 0.0;
  double CdS_booms_cand_m2 = 0.0;
  double d_CdS_booms_m2 = 0.0;

  double d_CdS_total_m2 = 0.0;

  bool ok = true;
  std::string message;
};

inline double group_CdS(const std::vector<DragItem>& items, const std::string& group_name) {
  double sum = 0.0;
  for (const auto& it : items) {
    it.validate();
    if (it.group == group_name) sum += it.effective_CdS();
  }
  if (!lift::bemt::is_finite(sum) || sum < 0.0) sum = 0.0;
  return sum;
}

inline BoomRemovalCheckOut check_boom_removal_consistency(const std::vector<DragItem>& baseline,
                                                          const std::vector<DragItem>& candidate,
                                                          const std::string& boom_group_name = "booms",
                                                          double tolerance_m2 = 1e-9) {
  BoomRemovalCheckOut out;
  LIFT_BEMT_REQUIRE(lift::bemt::is_finite(tolerance_m2) && tolerance_m2 >= 0.0,
                    lift::bemt::ErrorCode::InvalidInput, "tolerance invalid");

  const auto tb = compute_drag_totals(baseline);
  const auto tc = compute_drag_totals(candidate);

  out.CdS_booms_base_m2 = group_CdS(baseline, boom_group_name);
  out.CdS_booms_cand_m2 = group_CdS(candidate, boom_group_name);
  out.d_CdS_booms_m2 = out.CdS_booms_cand_m2 - out.CdS_booms_base_m2;

  out.d_CdS_total_m2 = tc.CdS_total_m2 - tb.CdS_total_m2;

  if (out.d_CdS_booms_m2 > tolerance_m2) {
    out.ok = false;
    out.message = "boom CdS increased; check grouping/tagging or sign errors";
    out.code = lift::bemt::ErrorCode::InvalidInput;
    return out;
  }

  const double boom_drop = -out.d_CdS_booms_m2;
  if (boom_drop > 1e-6 && out.d_CdS_total_m2 > 1e-6) {
    out.ok = false;
    out.message = "total CdS increased while booms decreased; verify other component CdS changes";
    out.code = lift::bemt::ErrorCode::InvalidInput;
    return out;
  }

  out.ok = true;
  out.message = "";
  out.code = lift::bemt::ErrorCode::Ok;
  return out;
}

} // namespace lift::physics
