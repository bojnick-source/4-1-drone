/*
===============================================================================
Fragment 3.1.31 — Concept Mass Delta Ledger (Itemized Δmass + CG/Inertia Deltas + Ratio Impact) (C++)
File: cpp/engine/closeout/mass_ledger.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::closeout {

// -----------------------------
// Basic math types
// -----------------------------
struct Vec3 final {
  double x = 0.0, y = 0.0, z = 0.0;
};

inline Vec3 add(const Vec3& a, const Vec3& b) noexcept { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 sub(const Vec3& a, const Vec3& b) noexcept { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline Vec3 mul(const Vec3& a, double s) noexcept { return {a.x * s, a.y * s, a.z * s}; }
inline double dot(const Vec3& a, const Vec3& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }

struct InertiaDiag final {
  double Ixx = 0.0, Iyy = 0.0, Izz = 0.0;
};

// -----------------------------
// Component representation
// -----------------------------
struct MassItem final {
  std::string id;      // stable id (e.g., "boom.L1", "gearbox.coax", etc.)
  std::string group;   // optional grouping (e.g., "propulsion", "structure")
  double mass_kg = 0.0;   // component mass
  Vec3 cg_m{};            // component CG position in vehicle frame

  // Optional local inertia about component CG (diagonal only).
  // If unknown, leave zeros; point-mass model will still compute global inertia via PA theorem.
  InertiaDiag I_local_kg_m2{};

  // Optional notes (non-essential)
  std::string note;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "MassItem.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(mass_kg) && mass_kg >= 0.0, lift::bemt::ErrorCode::InvalidInput, "MassItem.mass invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(cg_m.x) && lift::bemt::is_finite(cg_m.y) && lift::bemt::is_finite(cg_m.z),
                      lift::bemt::ErrorCode::InvalidInput, "MassItem.cg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(I_local_kg_m2.Ixx) && I_local_kg_m2.Ixx >= 0.0,
                      lift::bemt::ErrorCode::InvalidInput, "MassItem.Ixx invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(I_local_kg_m2.Iyy) && I_local_kg_m2.Iyy >= 0.0,
                      lift::bemt::ErrorCode::InvalidInput, "MassItem.Iyy invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(I_local_kg_m2.Izz) && I_local_kg_m2.Izz >= 0.0,
                      lift::bemt::ErrorCode::InvalidInput, "MassItem.Izz invalid");
  }
};

// -----------------------------
// Ledger totals
// -----------------------------
struct MassTotals final {
  double mass_kg = 0.0;
  Vec3 cg_m{};
  InertiaDiag I_kg_m2{};

  bool ok() const noexcept { return lift::bemt::is_finite(mass_kg) && mass_kg >= 0.0; }
};

inline InertiaDiag addI(const InertiaDiag& a, const InertiaDiag& b) noexcept {
  return {a.Ixx + b.Ixx, a.Iyy + b.Iyy, a.Izz + b.Izz};
}

inline InertiaDiag parallel_axis_diag(double m, const Vec3& r) noexcept {
  // I = m * (r^2 * I3 - r r^T)
  // Diagonal:
  // Ixx += m*(y^2 + z^2)
  // Iyy += m*(x^2 + z^2)
  // Izz += m*(x^2 + y^2)
  const double x2 = r.x * r.x;
  const double y2 = r.y * r.y;
  const double z2 = r.z * r.z;
  return {m * (y2 + z2), m * (x2 + z2), m * (x2 + y2)};
}

inline MassTotals compute_totals(const std::vector<MassItem>& items) {
  MassTotals t{};

  // Sum mass and first moment
  double msum = 0.0;
  Vec3 m1{0.0, 0.0, 0.0};

  for (const auto& it : items) {
    it.validate();
    if (it.mass_kg <= 0.0) continue;
    msum += it.mass_kg;
    m1 = add(m1, mul(it.cg_m, it.mass_kg));
  }

  t.mass_kg = lift::bemt::is_finite(msum) ? msum : 0.0;
  if (t.mass_kg > 0.0) {
    t.cg_m = mul(m1, 1.0 / t.mass_kg);
  } else {
    t.cg_m = {0.0, 0.0, 0.0};
  }

  // Inertia about total CG
  InertiaDiag Itot{0.0, 0.0, 0.0};
  for (const auto& it : items) {
    if (it.mass_kg <= 0.0) continue;
    const Vec3 r = sub(it.cg_m, t.cg_m);
    const InertiaDiag Ipa = parallel_axis_diag(it.mass_kg, r);
    Itot = addI(Itot, it.I_local_kg_m2);
    Itot = addI(Itot, Ipa);
  }

  if (!lift::bemt::is_finite(Itot.Ixx) || Itot.Ixx < 0.0) Itot.Ixx = 0.0;
  if (!lift::bemt::is_finite(Itot.Iyy) || Itot.Iyy < 0.0) Itot.Iyy = 0.0;
  if (!lift::bemt::is_finite(Itot.Izz) || Itot.Izz < 0.0) Itot.Izz = 0.0;

  t.I_kg_m2 = Itot;
  return t;
}

// -----------------------------
// Delta / comparison
// -----------------------------
struct MassDelta final {
  MassTotals base{};
  MassTotals cand{};

  double d_mass_kg = 0.0;
  Vec3 d_cg_m{};
  InertiaDiag d_I_kg_m2{};

  // Ratio impacts (caller supplies payload and/or gross)
  double payload_kg = 0.0;

  double ratio_payload_to_empty_base = 0.0;
  double ratio_payload_to_empty_cand = 0.0;
  double d_ratio_payload_to_empty = 0.0;

  double gross_base_kg = 0.0;
  double gross_cand_kg = 0.0;
  double ratio_payload_to_gross_base = 0.0;
  double ratio_payload_to_gross_cand = 0.0;

  void compute_ratios(double payload_in_kg) {
    payload_kg = payload_in_kg;
    if (base.mass_kg > 0.0) ratio_payload_to_empty_base = payload_kg / base.mass_kg;
    if (cand.mass_kg > 0.0) ratio_payload_to_empty_cand = payload_kg / cand.mass_kg;
    d_ratio_payload_to_empty = ratio_payload_to_empty_cand - ratio_payload_to_empty_base;

    gross_base_kg = base.mass_kg + payload_kg;
    gross_cand_kg = cand.mass_kg + payload_kg;

    if (gross_base_kg > 0.0) ratio_payload_to_gross_base = payload_kg / gross_base_kg;
    if (gross_cand_kg > 0.0) ratio_payload_to_gross_cand = payload_kg / gross_cand_kg;

    if (!lift::bemt::is_finite(ratio_payload_to_empty_base)) ratio_payload_to_empty_base = 0.0;
    if (!lift::bemt::is_finite(ratio_payload_to_empty_cand)) ratio_payload_to_empty_cand = 0.0;
    if (!lift::bemt::is_finite(d_ratio_payload_to_empty)) d_ratio_payload_to_empty = 0.0;
    if (!lift::bemt::is_finite(ratio_payload_to_gross_base)) ratio_payload_to_gross_base = 0.0;
    if (!lift::bemt::is_finite(ratio_payload_to_gross_cand)) ratio_payload_to_gross_cand = 0.0;
  }
};

inline MassDelta compare_ledgers(const std::vector<MassItem>& baseline,
                                 const std::vector<MassItem>& candidate,
                                 double payload_kg = 0.0) {
  MassDelta d{};
  d.base = compute_totals(baseline);
  d.cand = compute_totals(candidate);

  d.d_mass_kg = d.cand.mass_kg - d.base.mass_kg;
  d.d_cg_m = sub(d.cand.cg_m, d.base.cg_m);
  d.d_I_kg_m2 = {d.cand.I_kg_m2.Ixx - d.base.I_kg_m2.Ixx,
                 d.cand.I_kg_m2.Iyy - d.base.I_kg_m2.Iyy,
                 d.cand.I_kg_m2.Izz - d.base.I_kg_m2.Izz};

  if (!lift::bemt::is_finite(d.d_mass_kg)) d.d_mass_kg = 0.0;
  if (!lift::bemt::is_finite(d.d_cg_m.x)) d.d_cg_m.x = 0.0;
  if (!lift::bemt::is_finite(d.d_cg_m.y)) d.d_cg_m.y = 0.0;
  if (!lift::bemt::is_finite(d.d_cg_m.z)) d.d_cg_m.z = 0.0;
  if (!lift::bemt::is_finite(d.d_I_kg_m2.Ixx)) d.d_I_kg_m2.Ixx = 0.0;
  if (!lift::bemt::is_finite(d.d_I_kg_m2.Iyy)) d.d_I_kg_m2.Iyy = 0.0;
  if (!lift::bemt::is_finite(d.d_I_kg_m2.Izz)) d.d_I_kg_m2.Izz = 0.0;

  d.compute_ratios(payload_kg);
  return d;
}

// -----------------------------
// Itemized delta (join by id)
// -----------------------------
struct ItemDelta final {
  std::string id;
  std::string group;
  double mass_base_kg = 0.0;
  double mass_cand_kg = 0.0;
  double d_mass_kg = 0.0;
  Vec3 cg_base_m{};
  Vec3 cg_cand_m{};
  Vec3 d_cg_m{};
};

inline const MassItem* find_item(const std::vector<MassItem>& v, const std::string& id) noexcept {
  for (const auto& it : v)
    if (it.id == id) return &it;
  return nullptr;
}

inline std::vector<ItemDelta> itemized_deltas(const std::vector<MassItem>& baseline,
                                              const std::vector<MassItem>& candidate) {
  // Deterministic union of ids: baseline order, then candidate-only in candidate order
  std::vector<ItemDelta> out;
  out.reserve(baseline.size() + candidate.size());

  auto push_delta = [&](const std::string& id, const MassItem* b, const MassItem* c) {
    ItemDelta d;
    d.id = id;
    if (b) {
      d.group = b->group;
      d.mass_base_kg = b->mass_kg;
      d.cg_base_m = b->cg_m;
    }
    if (c) {
      if (d.group.empty()) d.group = c->group;
      d.mass_cand_kg = c->mass_kg;
      d.cg_cand_m = c->cg_m;
    }
    d.d_mass_kg = d.mass_cand_kg - d.mass_base_kg;
    d.d_cg_m = sub(d.cg_cand_m, d.cg_base_m);

    if (!lift::bemt::is_finite(d.mass_base_kg) || d.mass_base_kg < 0.0) d.mass_base_kg = 0.0;
    if (!lift::bemt::is_finite(d.mass_cand_kg) || d.mass_cand_kg < 0.0) d.mass_cand_kg = 0.0;
    if (!lift::bemt::is_finite(d.d_mass_kg)) d.d_mass_kg = 0.0;

    out.push_back(std::move(d));
  };

  for (const auto& b : baseline) {
    const auto* c = find_item(candidate, b.id);
    push_delta(b.id, &b, c);
  }
  for (const auto& c : candidate) {
    const auto* b = find_item(baseline, c.id);
    if (!b) push_delta(c.id, nullptr, &c);
  }

  return out;
}

} // namespace lift::closeout
