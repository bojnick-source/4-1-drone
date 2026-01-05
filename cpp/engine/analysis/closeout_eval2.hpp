#pragma once

#include <optional>

#include "engine/analysis/closeout_types.hpp"

namespace lift {

// Configurable evaluation thresholds.
// If a threshold is not set, evaluator will only check for presence + basic validity (e.g., > 0).
struct CloseoutEvalConfig {
  // If set: delta_mass_total_kg must be <= this to be "Go".
  std::optional<double> max_delta_mass_total_kg;

  // If set: disk_area_m2 must be >= this to be "Go".
  std::optional<double> min_disk_area_m2;

  // If set: power_hover_kw must be <= this to be "Go".
  std::optional<double> max_power_hover_kw;

  // If true: mass breakdown (mass_items) is REQUIRED for mass gate to be Go.
  // If mass_items is empty => NeedsData even if delta_mass_total_kg is finite.
  bool require_mass_breakdown = true;
};

// Mutates report in-place:
// - Preserves NaN-as-unset semantics
// - Produces deterministic gates + deterministic issues (stable order)
// - Never “defaults” missing numeric values to 0.0
void evaluate_closeout(CloseoutReport& report, const CloseoutEvalConfig& cfg);

// Convenience default (no thresholds, still enforces require_mass_breakdown=true).
inline void evaluate_closeout(CloseoutReport& report) {
  evaluate_closeout(report, CloseoutEvalConfig{});
}

}  // namespace lift

