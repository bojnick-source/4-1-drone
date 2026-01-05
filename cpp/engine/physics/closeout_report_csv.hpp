/*
===============================================================================
Fragment 3.1.20 — GO/NO-GO CSV Layer (Reasons Flattened) (C++)
File: closeout_report_csv.hpp
===============================================================================
*/

#pragma once

#include "bemt_closeout_csv.hpp"
#include "closeout_thresholds.hpp"

#include <string>
#include <vector>

namespace lift::bemt {

// Flattened GO/NO-GO CSV for backlog/debugging.
// Columns: case_id,status,reasons_count,reasons_keys,reasons_messages
std::string gonogo_csv_header();
std::string gonogo_csv_row(const GoNoGoReport& r);
std::string gonogo_csv(const std::vector<GoNoGoReport>& rs);

// Helper: evaluate rows then emit CSV.
std::vector<GoNoGoReport> evaluate_all(const std::vector<CloseoutRow>& rows,
                                       const GoNoGoEvaluator& eval,
                                       double A_total_m2_override = -1.0,
                                       double delta_mass_kg = 0.0);

} // namespace lift::bemt
