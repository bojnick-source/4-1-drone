/*
===============================================================================
Fragment 3.1.19 — GO/NO-GO Thresholds + Evaluator (Numerical Gates) (C++)
File: closeout_thresholds.hpp
===============================================================================
*/

#pragma once

#include "bemt_closeout_csv.hpp"
#include "bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace lift::bemt {

// GO/NO-GO status
enum class GoNoGoStatus : std::uint8_t {
    Go = 0,
    NoGo = 1
};

// One reason entry (key is machine-friendly, message is human-friendly).
struct GoNoGoReason final {
    std::string key;
    std::string message;
};

// Final report per case.
struct GoNoGoReport final {
    std::string case_id;
    GoNoGoStatus status = GoNoGoStatus::Go;
    std::vector<GoNoGoReason> reasons;
};

// Threshold set (0 disables a gate unless otherwise noted).
struct GoNoGoThresholds final {
    // 1) Δmass gate (concept delta vs baseline).
    double delta_mass_max_kg = 0.0;   // disable if 0

    // 2) Disk area gate (A_total across independent disks).
    double A_total_min_m2 = 0.0;      // disable if 0

    // 2b) Disk loading gate (DL = T/A) (per-rotor or total depending on use).
    double disk_loading_max_N_m2 = 0.0; // disable if 0

    // 2c) Hover power gate (actual, from BEMT closeout).
    double hover_power_max_W = 0.0;   // disable if 0

    // 2d) Hover FM minimum.
    double fm_min = 0.0;              // disable if 0

    // 3) Convergence gates
    std::size_t inflow_iters_max = 0; // disable if 0
    std::size_t trim_iters_max = 0;   // disable if 0

    // 4) Forward gates (if forward run exists)
    double fwd_power_max_W = 0.0;     // disable if 0
    double fwd_T_min_N = 0.0;         // disable if 0

    // Hard sanity guards (always on)
    double min_positive = 1e-9;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(delta_mass_max_kg) && delta_mass_max_kg >= 0.0, ErrorCode::InvalidConfig, "delta_mass_max_kg invalid");
        LIFT_BEMT_REQUIRE(is_finite(A_total_min_m2) && A_total_min_m2 >= 0.0, ErrorCode::InvalidConfig, "A_total_min_m2 invalid");
        LIFT_BEMT_REQUIRE(is_finite(disk_loading_max_N_m2) && disk_loading_max_N_m2 >= 0.0, ErrorCode::InvalidConfig, "disk_loading_max_N_m2 invalid");
        LIFT_BEMT_REQUIRE(is_finite(hover_power_max_W) && hover_power_max_W >= 0.0, ErrorCode::InvalidConfig, "hover_power_max_W invalid");
        LIFT_BEMT_REQUIRE(is_finite(fm_min) && fm_min >= 0.0 && fm_min <= 1.0, ErrorCode::InvalidConfig, "fm_min invalid");

        LIFT_BEMT_REQUIRE(is_finite(fwd_power_max_W) && fwd_power_max_W >= 0.0, ErrorCode::InvalidConfig, "fwd_power_max_W invalid");
        LIFT_BEMT_REQUIRE(is_finite(fwd_T_min_N) && fwd_T_min_N >= 0.0, ErrorCode::InvalidConfig, "fwd_T_min_N invalid");

        LIFT_BEMT_REQUIRE(is_finite(min_positive) && min_positive > 0.0, ErrorCode::InvalidConfig, "min_positive invalid");
    }
};

// Evaluator: turns CloseoutRow + optional external deltas into GO/NO-GO reasons.
class GoNoGoEvaluator final {
public:
    explicit GoNoGoEvaluator(GoNoGoThresholds t) : t_(std::move(t)) { t_.validate(); }

    // A_total_m2_override:
    //  - if >0, used as A_total for area gate (sum of independent disks)
    //  - else uses row.A_m2 as best available proxy (single rotor)
    //
    // delta_mass_kg:
    //  - external concept delta (mass ledger); can be 0 if unknown.
    GoNoGoReport evaluate(const CloseoutRow& row,
                          double A_total_m2_override = -1.0,
                          double delta_mass_kg = 0.0) const;

    const GoNoGoThresholds& thresholds() const noexcept { return t_; }

private:
    void add_(GoNoGoReport& r, const char* key, const std::string& msg) const;

private:
    GoNoGoThresholds t_;
};

} // namespace lift::bemt
