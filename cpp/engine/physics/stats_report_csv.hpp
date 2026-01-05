/*
===============================================================================
Fragment 3.1.15 — Uncertainty Report CSV (Moments + Quantiles + CDF Probe)
File: stats_report_csv.hpp
===============================================================================
*/

#pragma once
#include "engine/physics/stats_hooks.hpp"

#include <string>

namespace lift::bemt::stats {

// Column header for the uncertainty CSV export.
std::string uncertainty_csv_header();

// Emit a single CSV row for the given uncertainty report.
// thrust_probe_N / power_probe_W are optional CDF probe points (emit NaN/0 to skip).
std::string uncertainty_csv_row(const UncertaintyReport& r,
                                double thrust_probe_N,
                                double power_probe_W);

// Convenience: header + single row.
std::string uncertainty_csv(const UncertaintyReport& r,
                            double thrust_probe_N,
                            double power_probe_W);

} // namespace lift::bemt::stats
