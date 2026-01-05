/*
===============================================================================
Fragment 3.2.03 — CFD Calibration Apply Hook (Correct BEMT Outputs in Closeout)
File: cfd_apply.cpp
===============================================================================
*/

#include "cfd_apply.hpp"

namespace lift::bemt {

namespace {
bool valid_corr(double v) {
    return is_finite(v) && v > 0.0;
}
} // namespace

std::vector<CloseoutRowCorrected> apply_cfd_calibration(const std::vector<CloseoutRow>& rows,
                                                        const CfdCalibrationTable& cal) {
    std::vector<CloseoutRowCorrected> out;
    out.reserve(rows.size());

    for (const auto& r : rows) {
        CloseoutRowCorrected c{};
        static_cast<CloseoutRow&>(c) = r; // copy base fields

        double corr_T = 1.0;
        double corr_P = 1.0;

        if (!r.case_id.empty()) {
            const CfdCalibrationEntry* e = cal.find(r.case_id);
            if (e && e->code == ErrorCode::Ok) {
                if (valid_corr(e->correction_thrust)) corr_T = e->correction_thrust;
                if (valid_corr(e->correction_power))  corr_P = e->correction_power;
            }
        }

        c.cfd_corr_T = corr_T;
        c.cfd_corr_P = corr_P;

        // Apply to hover
        c.corr_hover_T_N = r.hover_T_N * corr_T;
        c.corr_hover_P_W = r.hover_P_W * corr_P;

        // Apply to forward (if present)
        c.corr_fwd_T_N = r.fwd_T_N * corr_T;
        c.corr_fwd_P_W = r.fwd_P_W * corr_P;

        out.push_back(std::move(c));
    }

    return out;
}

} // namespace lift::bemt
