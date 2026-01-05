#pragma once
// ============================================================================
// BEMT Uncertainty Runner Data Structures
// File: cpp/engine/physics/bemt_uncertainty_runner.hpp
// ============================================================================
//
// Purpose:
// - Define data structures for uncertainty quantification results.
// - Used by stats_report_csv to serialize uncertainty summaries.
//
// ============================================================================

#include <string>
#include <vector>

namespace lift::bemt {

struct BemtUncSummary {
    std::string metric_key;
    std::string units;
    double mean = 0.0;
    double stdev = 0.0;
    double min = 0.0;
    double max = 0.0;
    double prob_meets_threshold = 0.0;
    std::vector<double> q;  // quantile values
};

struct BemtUncertaintyReport {
    std::vector<BemtUncSummary> summaries;

    void validate() const {
        // Placeholder for validation logic
        // Could check for NaN values, empty summaries, etc.
    }
};

} // namespace lift::bemt
