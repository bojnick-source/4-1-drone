// ============================================================================
// Fragment 3.3.02 — CDF/Probability CSV Report (Quantiles, Exceedance, Moments)
// File: cdf_report_csv.hpp
// ============================================================================
//
// Purpose:
// - Serialize probability summaries to CSV for closeout/stat reporting.
// - Uses EmpiricalCdf:
//   - moments (min/max/mean/std)
//   - quantiles (p10,p50,p90,p95,p99)
//   - exceedance probabilities for key thresholds
//
// Schema is stable and explicit.
// ============================================================================

#pragma once
#include "cdf.hpp"

#include <limits>
#include <string>
#include <vector>

namespace lift::bemt::prob {

struct ProbSummary final {
    std::string case_id;
    std::string metric_name;  // e.g., "hover_power_W", "FM", "thrust_margin_N"

    std::size_t n = 0;

    // Moments
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    double stddev = 0.0;

    // Quantiles
    double p10 = 0.0;
    double p50 = 0.0;
    double p90 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;

    // Exceedance probabilities (optional; if threshold unset, set to NaN)
    double thr1 = std::numeric_limits<double>::quiet_NaN();
    double p_ge_thr1 = std::numeric_limits<double>::quiet_NaN();

    double thr2 = std::numeric_limits<double>::quiet_NaN();
    double p_ge_thr2 = std::numeric_limits<double>::quiet_NaN();
};

ProbSummary summarize(const std::string& case_id,
                      const std::string& metric_name,
                      const EmpiricalCdf& cdf,
                      double thr1 = std::numeric_limits<double>::quiet_NaN(),
                      double thr2 = std::numeric_limits<double>::quiet_NaN());

std::string prob_csv_header();
std::string prob_csv_row(const ProbSummary& s);
std::string prob_csv(const std::vector<ProbSummary>& ss);

} // namespace lift::bemt::prob
