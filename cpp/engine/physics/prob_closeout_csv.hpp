// ============================================================================
// Fragment 3.3.05 — Probability Closeout CSV (prob_closeout.csv)
// File: prob_closeout_csv.hpp
// ============================================================================
//
// Purpose:
// - Emit probability summaries and probabilistic GO/NO-GO outcomes in a stable CSV.
// - Intended output file: prob_closeout.csv
//
// Rows are "case_id + metric summary", one row per metric per case.
// Separate report CSV for gate decisions is also included.
//
// ============================================================================

#pragma once
#include "cdf_report_csv.hpp"
#include "prob_gates.hpp"

#include <string>
#include <vector>

namespace lift::bemt::prob {

// Gate decision CSV
std::string prob_gate_csv_header();
std::string prob_gate_csv_row(const ProbGateReport& r);
std::string prob_gate_csv(const std::vector<ProbGateReport>& rs);

// Summary CSV is reused from cdf_report_csv (prob_csv)

} // namespace lift::bemt::prob
