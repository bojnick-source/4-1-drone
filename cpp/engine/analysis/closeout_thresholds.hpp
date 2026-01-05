#pragma once
// ============================================================================
// Closeout Thresholds
// File: cpp/engine/analysis/closeout_thresholds.hpp
// ============================================================================
//
// Purpose:
// - Define threshold configurations for closeout gating.
// - Provide default and strict threshold sets.
//
// ============================================================================

#include "engine/analysis/closeout_types.hpp"

#include <string>

namespace lift::analysis {

struct CloseoutThresholds {
    double max_delta_mass_kg = lift::kUnset;
    double min_A_total_m2 = lift::kUnset;
    double min_parasite_power_reduction_pct = lift::kUnset;
    double min_yaw_margin_ratio = lift::kUnset;
    double min_phase_tolerance_deg = lift::kUnset;
    double max_latency_ms = lift::kUnset;
    double max_time_increase_pct = lift::kUnset;
};

// Default threshold set (more permissive)
inline CloseoutThresholds default_closeout_thresholds() {
    CloseoutThresholds t;
    t.max_delta_mass_kg = 2.0;
    t.min_A_total_m2 = 1.5;
    t.min_parasite_power_reduction_pct = 3.0;
    t.min_yaw_margin_ratio = 1.05;
    t.max_time_increase_pct = 5.0;
    return t;
}

// Strict threshold set (more conservative)
inline CloseoutThresholds strict_closeout_thresholds() {
    CloseoutThresholds t;
    t.max_delta_mass_kg = 1.5;
    t.min_A_total_m2 = 1.6;
    t.min_parasite_power_reduction_pct = 5.0;
    t.min_yaw_margin_ratio = 1.10;
    t.max_time_increase_pct = 2.0;
    return t;
}

// Validation helper
inline void validate_closeout_thresholds_or_throw(const CloseoutThresholds& t, const std::string& label) {
    // Basic validation - ensure thresholds are reasonable if set
    // For now, just a placeholder that could check for invalid combinations
    (void)t;
    (void)label;
}

} // namespace lift::analysis
