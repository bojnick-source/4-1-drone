/*
===============================================================================
Fragment 3.2.03 — CFD Calibration Apply Hook (Correct BEMT Outputs in Closeout)
File: cfd_apply.hpp
===============================================================================
*/

#pragma once
#include "bemt_closeout_csv.hpp"
#include "cfd_results.hpp"
#include "bemt_require.hpp"

#include <string>
#include <vector>

namespace lift::bemt {

struct CloseoutRowCorrected final : public CloseoutRow {
    // Multipliers used (1.0 if none found)
    double cfd_corr_T = 1.0;
    double cfd_corr_P = 1.0;

    // Corrected outputs
    double corr_hover_T_N = 0.0;
    double corr_hover_P_W = 0.0;

    double corr_fwd_T_N = 0.0;
    double corr_fwd_P_W = 0.0;
};

// Apply calibration table to closeout rows.
std::vector<CloseoutRowCorrected> apply_cfd_calibration(const std::vector<CloseoutRow>& rows,
                                                        const CfdCalibrationTable& cal);

} // namespace lift::bemt
