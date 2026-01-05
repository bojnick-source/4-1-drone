/*
===============================================================================
Fragment 3.2.07 — CFD Closeout Gates (Calibration Quality + Drift Checks)
File: cfd_gates.cpp
===============================================================================
*/

#include "cfd_gates.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <limits>

namespace lift::bemt {

namespace {
inline double rel_err(double num, double den) {
    if (!(is_finite(num) && is_finite(den) && den > 0.0)) return std::numeric_limits<double>::infinity();
    return std::abs(num - den) / den;
}
} // namespace

CfdGateResult gate_cfd_calibration(const CfdCalibrationTable& table,
                                   const CfdGateThresholds& thr) {
    thr.validate();

    CfdGateResult res{};
    res.total = table.entries.size();

    for (const auto& e_in : table.entries) {
        CfdCalibrationEntry e = e_in; // copy to annotate if rejected

        // Basic sanity on correction clamp
        if (!is_finite(e.correction_thrust) || !is_finite(e.correction_power) ||
            e.correction_thrust < thr.min_corr || e.correction_thrust > thr.max_corr ||
            e.correction_power  < thr.min_corr || e.correction_power  > thr.max_corr) {
            e.code = ErrorCode::InvalidInput;
            e.message = "Correction outside gating bounds";
            res.rejected_entries.push_back(std::move(e));
            continue;
        }

        // Relative error checks (optional: thresholds == 0 disables)
        if (thr.max_rel_err_thrust > 0.0) {
            const double errT = rel_err(e.T_cfd_N, e.T_bemt_N);
            if (!(errT <= thr.max_rel_err_thrust)) {
                e.code = ErrorCode::InvalidInput;
                e.message = "Thrust relative error too high";
                res.rejected_entries.push_back(std::move(e));
                continue;
            }
        }

        if (thr.max_rel_err_power > 0.0) {
            const double errP = rel_err(e.P_cfd_W, e.P_bemt_W);
            if (!(errP <= thr.max_rel_err_power)) {
                e.code = ErrorCode::InvalidInput;
                e.message = "Power relative error too high";
                res.rejected_entries.push_back(std::move(e));
                continue;
            }
        }

        res.accepted.push_back(std::move(e));
    }

    res.ok = res.accepted.size();
    res.rejected = res.rejected_entries.size();

    if (res.ok < thr.min_ok_cases) {
        res.code = ErrorCode::NonConverged;
        res.message = "Insufficient CFD samples after gating";
        res.accepted.clear();
    } else {
        res.code = ErrorCode::Ok;
        res.message = "OK";
    }

    return res;
}

} // namespace lift::bemt
