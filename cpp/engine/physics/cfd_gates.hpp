/*
===============================================================================
Fragment 3.2.07 — CFD Closeout Gates (Calibration Quality + Drift Checks)
File: cfd_gates.hpp
===============================================================================
*/

#pragma once
#include "cfd_results.hpp"
#include "bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::bemt {

struct CfdGateThresholds final {
    // Relative agreement thresholds (0 disables)
    double max_rel_err_thrust = 0.15; // |T_cfd - T_bemt| / T_bemt
    double max_rel_err_power  = 0.20; // |P_cfd - P_bemt| / P_bemt

    // Correction clamp sanity (extra protection beyond ingest clamp)
    double min_corr = 0.6;
    double max_corr = 1.8;

    // Minimum usable CFD cases required before enabling calibration
    std::size_t min_ok_cases = 5;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(max_rel_err_thrust) && max_rel_err_thrust >= 0.0 && max_rel_err_thrust < 5.0,
                          ErrorCode::InvalidConfig, "max_rel_err_thrust invalid");
        LIFT_BEMT_REQUIRE(is_finite(max_rel_err_power) && max_rel_err_power >= 0.0 && max_rel_err_power < 5.0,
                          ErrorCode::InvalidConfig, "max_rel_err_power invalid");
        LIFT_BEMT_REQUIRE(is_finite(min_corr) && is_finite(max_corr) && min_corr > 0.0 && max_corr > min_corr,
                          ErrorCode::InvalidConfig, "corr bounds invalid");
        LIFT_BEMT_REQUIRE(min_ok_cases <= 1000000, ErrorCode::InvalidConfig, "min_ok_cases invalid");
    }
};

struct CfdGateResult final {
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    std::size_t total = 0;
    std::size_t ok = 0;
    std::size_t rejected = 0;

    // Entries passing gates
    std::vector<CfdCalibrationEntry> accepted;

    // Entries rejected (with message + code already set on entry)
    std::vector<CfdCalibrationEntry> rejected_entries;
};

// Apply gates to calibration table; returns accepted-only entries plus status.
// If ok < min_ok_cases, returns code=NonConverged and accepted empty.
CfdGateResult gate_cfd_calibration(const CfdCalibrationTable& table,
                                   const CfdGateThresholds& thr);

} // namespace lift::bemt
