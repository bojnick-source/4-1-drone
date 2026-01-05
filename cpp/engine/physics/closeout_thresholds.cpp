/*
===============================================================================
Fragment 3.1.19 — GO/NO-GO Thresholds + Evaluator (Numerical Gates) (C++)
File: closeout_thresholds.cpp
===============================================================================
*/

#include "closeout_thresholds.hpp"

#include <sstream>

namespace lift::bemt {

void GoNoGoEvaluator::add_(GoNoGoReport& r, const char* key, const std::string& msg) const {
    r.status = GoNoGoStatus::NoGo;
    GoNoGoReason rr;
    rr.key = key ? std::string(key) : std::string("unknown");
    rr.message = msg;
    r.reasons.push_back(std::move(rr));
}

GoNoGoReport GoNoGoEvaluator::evaluate(const CloseoutRow& row,
                                      double A_total_m2_override,
                                      double delta_mass_kg) const {
    GoNoGoReport rep;
    rep.case_id = row.case_id;
    rep.status = GoNoGoStatus::Go;
    rep.reasons.clear();
    rep.reasons.reserve(16);

    // ---- Baseline validity checks (always on)
    if (row.hover_code != ErrorCode::Ok) {
        std::ostringstream os;
        os << "Hover solver code != Ok (" << static_cast<unsigned>(row.hover_code) << ")";
        add_(rep, "hover_solve_failed", os.str());
        // Keep evaluating to collect all reasons.
    }

    if (!is_finite(row.hover_T_N) || row.hover_T_N < 0.0 ||
        !is_finite(row.hover_P_W) || row.hover_P_W < 0.0 ||
        !is_finite(row.A_m2) || row.A_m2 < 0.0) {
        add_(rep, "hover_metrics_invalid", "Hover outputs contain invalid/non-finite values");
    }

    // ---- 1) Δmass gate
    if (t_.delta_mass_max_kg > 0.0) {
        if (!is_finite(delta_mass_kg)) {
            add_(rep, "delta_mass_nan", "Δmass is non-finite (mass ledger invalid)");
        } else if (delta_mass_kg > t_.delta_mass_max_kg) {
            std::ostringstream os;
            os << "Δmass " << delta_mass_kg << " kg exceeds max " << t_.delta_mass_max_kg << " kg";
            add_(rep, "delta_mass_exceeds", os.str());
        }
    }

    // ---- 2) Disk area gate (A_total)
    const double A_total = (is_finite(A_total_m2_override) && A_total_m2_override > 0.0)
                         ? A_total_m2_override
                         : row.A_m2;

    if (t_.A_total_min_m2 > 0.0) {
        if (!is_finite(A_total) || A_total <= 0.0) {
            add_(rep, "area_invalid", "A_total is invalid/zero");
        } else if (A_total < t_.A_total_min_m2) {
            std::ostringstream os;
            os << "A_total " << A_total << " m^2 is below min " << t_.A_total_min_m2 << " m^2";
            add_(rep, "area_below_min", os.str());
        }
    }

    // ---- 2b) Disk loading gate
    if (t_.disk_loading_max_N_m2 > 0.0) {
        const double DL = (A_total > t_.min_positive) ? safe_div(row.hover_T_N, A_total, 0.0) : 0.0;
        if (!is_finite(DL) || DL <= 0.0) {
            add_(rep, "disk_loading_invalid", "Disk loading is invalid/zero");
        } else if (DL > t_.disk_loading_max_N_m2) {
            std::ostringstream os;
            os << "Disk loading " << DL << " N/m^2 exceeds max " << t_.disk_loading_max_N_m2 << " N/m^2";
            add_(rep, "disk_loading_exceeds", os.str());
        }
    }

    // ---- 2c) Hover power gate
    if (t_.hover_power_max_W > 0.0) {
        if (!is_finite(row.hover_P_W)) {
            add_(rep, "hover_power_nan", "Hover power is non-finite");
        } else if (row.hover_P_W > t_.hover_power_max_W) {
            std::ostringstream os;
            os << "Hover power " << row.hover_P_W << " W exceeds max " << t_.hover_power_max_W << " W";
            add_(rep, "hover_power_exceeds", os.str());
        }
    }

    // ---- 2d) FM minimum gate
    if (t_.fm_min > 0.0) {
        if (!is_finite(row.hover_FM) || row.hover_FM <= 0.0) {
            add_(rep, "fm_invalid", "FM is invalid/zero");
        } else if (row.hover_FM < t_.fm_min) {
            std::ostringstream os;
            os << "FM " << row.hover_FM << " below min " << t_.fm_min;
            add_(rep, "fm_below_min", os.str());
        }
    }

    // ---- 3) Convergence gates
    if (t_.inflow_iters_max > 0) {
        // We don't store it in CloseoutRow yet; treat missing as pass.
        // Gate must be wired when closeout row includes inflow_iters.
        // Placeholder: no-op.
    }

    if (t_.trim_iters_max > 0) {
        // Same note as above.
        // Placeholder: no-op.
    }

    // ---- 4) Forward gates (only if forward was run)
    const bool forward_present = (row.V_inplane_mps > 0.0) || (row.fwd_P_W > 0.0) || (row.fwd_T_N > 0.0) || (row.fwd_code != ErrorCode::Ok);
    if (forward_present) {
        if (row.fwd_code != ErrorCode::Ok) {
            std::ostringstream os;
            os << "Forward solver code != Ok (" << static_cast<unsigned>(row.fwd_code) << ")";
            add_(rep, "fwd_solve_failed", os.str());
        }

        if (t_.fwd_power_max_W > 0.0) {
            if (!is_finite(row.fwd_P_W)) {
                add_(rep, "fwd_power_nan", "Forward power is non-finite");
            } else if (row.fwd_P_W > t_.fwd_power_max_W) {
                std::ostringstream os;
                os << "Forward power " << row.fwd_P_W << " W exceeds max " << t_.fwd_power_max_W << " W";
                add_(rep, "fwd_power_exceeds", os.str());
            }
        }

        if (t_.fwd_T_min_N > 0.0) {
            if (!is_finite(row.fwd_T_N)) {
                add_(rep, "fwd_thrust_nan", "Forward thrust is non-finite");
            } else if (row.fwd_T_N < t_.fwd_T_min_N) {
                std::ostringstream os;
                os << "Forward thrust " << row.fwd_T_N << " N below min " << t_.fwd_T_min_N << " N";
                add_(rep, "fwd_thrust_below_min", os.str());
            }
        }
    }

    // If no reasons, keep GO.
    if (!rep.reasons.empty()) {
        rep.status = GoNoGoStatus::NoGo;
    } else {
        rep.status = GoNoGoStatus::Go;
    }

    return rep;
}

} // namespace lift::bemt
