/*
===============================================================================
Fragment 3.2.04 — Corrected Closeout CSV (Adds corrected_* columns)
File: cfd_closeout_csv.hpp
===============================================================================
*/

#pragma once
#include "cfd_apply.hpp"

#include <string>
#include <vector>

namespace lift::bemt {

// CSV schema = original closeout row + calibration + corrected fields.
// This lets downstream quickly compare raw vs corrected.
std::string closeout_corrected_csv_header();
std::string closeout_corrected_csv_row(const CloseoutRowCorrected& r);
std::string closeout_corrected_csv(const std::vector<CloseoutRowCorrected>& rows);

} // namespace lift::bemt
