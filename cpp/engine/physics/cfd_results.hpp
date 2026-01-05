/*
===============================================================================
Fragment 3.2.02 — CFD Results Ingestor (cfd_results.csv → Calibration Factors) (C++)
File: cfd_results.hpp
===============================================================================
*/

#pragma once
#include "bemt_error.hpp"
#include "bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace lift::bemt {

struct CfdCalibrationEntry final {
    std::string case_id;
    std::string job_id;

    // Raw CFD values
    double T_cfd_N = 0.0;
    double P_cfd_W = 0.0;

    // Reference BEMT values used for ratio
    double T_bemt_N = 0.0;
    double P_bemt_W = 0.0;

    // Derived multipliers (clamped)
    double correction_thrust = 1.0;
    double correction_power  = 1.0;

    // Status
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    void validate() const {
        LIFT_BEMT_REQUIRE(!case_id.empty(), ErrorCode::InvalidInput, "CfdCalibrationEntry.case_id empty");
        LIFT_BEMT_REQUIRE(is_finite(T_cfd_N) && T_cfd_N >= 0.0, ErrorCode::InvalidInput, "T_cfd_N invalid");
        LIFT_BEMT_REQUIRE(is_finite(P_cfd_W) && P_cfd_W >= 0.0, ErrorCode::InvalidInput, "P_cfd_W invalid");
        LIFT_BEMT_REQUIRE(is_finite(T_bemt_N) && T_bemt_N >= 0.0, ErrorCode::InvalidInput, "T_bemt_N invalid");
        LIFT_BEMT_REQUIRE(is_finite(P_bemt_W) && P_bemt_W >= 0.0, ErrorCode::InvalidInput, "P_bemt_W invalid");
        LIFT_BEMT_REQUIRE(is_finite(correction_thrust) && correction_thrust > 0.0, ErrorCode::InvalidInput, "correction_thrust invalid");
        LIFT_BEMT_REQUIRE(is_finite(correction_power) && correction_power > 0.0, ErrorCode::InvalidInput, "correction_power invalid");
    }
};

struct CfdCalibrationTable final {
    std::vector<CfdCalibrationEntry> entries;

    // Fast lookup: case_id -> index
    std::unordered_map<std::string, std::size_t> by_case;

    void rebuild_index() {
        by_case.clear();
        by_case.reserve(entries.size());
        for (std::size_t i = 0; i < entries.size(); ++i) {
            if (!entries[i].case_id.empty()) by_case[entries[i].case_id] = i;
        }
    }

    const CfdCalibrationEntry* find(const std::string& case_id) const {
        auto it = by_case.find(case_id);
        if (it == by_case.end()) return nullptr;
        return &entries[it->second];
    }
};

struct CfdIngestConfig final {
    // Clamp multipliers to avoid poisoning optimizer due to a single bad CFD run.
    double min_corr = 0.5;
    double max_corr = 2.0;

    // If true, require positive BEMT reference values for ratio computation.
    bool require_bemt_reference = true;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(min_corr) && is_finite(max_corr) && min_corr > 0.0 && max_corr > min_corr,
                          ErrorCode::InvalidConfig, "CfdIngestConfig corr clamp invalid");
    }
};

// Parse CFD results CSV and build calibration factors.
// bemt_reference maps case_id -> (T_bemt_N, P_bemt_W) as maps supplied by caller.
CfdCalibrationTable ingest_cfd_results_csv(const std::string& cfd_csv,
                                           const std::unordered_map<std::string, double>& bemt_T_ref,
                                           const std::unordered_map<std::string, double>& bemt_P_ref,
                                           const CfdIngestConfig& cfg);

} // namespace lift::bemt
