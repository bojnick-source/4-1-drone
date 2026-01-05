// ============================================================================
// Fragment 3.1.07 — BEMT Station Grid Hardening (Monotonic r checks + robust dr computation) (C++)
// File: bemt_station_grid.hpp
// ============================================================================
//
// Purpose:
// - Robust “station grid” utilities for BEMT:
//     * Validate strictly increasing radial stations
//     * Compute per-station dr (end-capped, midpoint average interior)
// - Eliminates repeated, inconsistent dr logic across solver variants.
// - Designed to be used by bemt_solver hover/forward integration loops.
//
// dr policy:
// - n < 2 -> all dr = 0
// - i == 0 -> dr = max(0, r[1] - r[0])
// - i == n-1 -> dr = max(0, r[n-1] - r[n-2])
// - interior -> dr = 0.5 * ((r[i+1]-r[i]) + (r[i]-r[i-1]))
//
// Safety:
// - Enforces strictly increasing r by default (can be relaxed).
// - Clips negative dr to 0 (should not happen if r monotonic).
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"
#include "bemt_safety.hpp"

#include <cstddef>
#include <vector>

namespace lift::bemt {

struct StationGridConfig final {
    bool require_strictly_increasing = true;
    bool allow_equal = false; // if require_strictly_increasing=false, equal allowed only when true
    double min_dr_m = 0.0;    // optional clamp to prevent near-zero dr in integration
    double max_dr_m = 1e9;    // optional clamp

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(min_dr_m) && min_dr_m >= 0.0, ErrorCode::InvalidConfig, "StationGridConfig.min_dr invalid");
        LIFT_BEMT_REQUIRE(is_finite(max_dr_m) && max_dr_m > min_dr_m, ErrorCode::InvalidConfig, "StationGridConfig.max_dr invalid");
    }
};

inline void validate_station_r(const std::vector<double>& r_m, const StationGridConfig& cfg_in = {}) {
    StationGridConfig cfg = cfg_in;
    cfg.validate();

    LIFT_BEMT_REQUIRE(!r_m.empty(), ErrorCode::InvalidInput, "station r array empty");
    for (std::size_t i = 0; i < r_m.size(); ++i) {
        LIFT_BEMT_REQUIRE(is_finite(r_m[i]) && r_m[i] >= 0.0, ErrorCode::InvalidInput, "station r invalid");
        if (i == 0) continue;

        if (cfg.require_strictly_increasing) {
            LIFT_BEMT_REQUIRE(r_m[i] > r_m[i - 1], ErrorCode::InvalidInput, "station r must be strictly increasing");
        } else {
            if (cfg.allow_equal) {
                LIFT_BEMT_REQUIRE(r_m[i] >= r_m[i - 1], ErrorCode::InvalidInput, "station r must be non-decreasing");
            } else {
                LIFT_BEMT_REQUIRE(r_m[i] > r_m[i - 1], ErrorCode::InvalidInput, "station r must be increasing (no equals)");
            }
        }
    }
}

inline std::vector<double> compute_station_dr(const std::vector<double>& r_m, const StationGridConfig& cfg_in = {}) {
    StationGridConfig cfg = cfg_in;
    cfg.validate();

    validate_station_r(r_m, cfg);

    const std::size_t n = r_m.size();
    std::vector<double> dr(n, 0.0);

    if (n < 2) return dr;

    for (std::size_t i = 0; i < n; ++i) {
        double d = 0.0;
        if (i == 0) {
            d = r_m[1] - r_m[0];
        } else if (i + 1 >= n) {
            d = r_m[n - 1] - r_m[n - 2];
        } else {
            const double d1 = r_m[i] - r_m[i - 1];
            const double d2 = r_m[i + 1] - r_m[i];
            d = 0.5 * (d1 + d2);
        }

        if (!is_finite(d) || d < 0.0) d = 0.0;

        // Optional clamps
        d = clamp(d, cfg.min_dr_m, cfg.max_dr_m);

        dr[i] = d;
    }

    return dr;
}

} // namespace lift::bemt
