// ============================================================================
// Fragment 3.3.02 — CDF/Probability CSV Report (Quantiles, Exceedance, Moments)
// File: cdf_report_csv.cpp
// ============================================================================

#include "cdf_report_csv.hpp"

#include <iomanip>
#include <limits>
#include <sstream>

namespace lift::bemt::prob {

namespace {

std::string esc_csv(const std::string& s) {
    bool need = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') { need = true; break; }
    }
    if (!need) return s;

    std::string out;
    out.reserve(s.size() + 8);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.append("\"\"");
        else out.push_back(c);
    }
    out.push_back('"');
    return out;
}

void fill_moments(ProbSummary& s, const EmpiricalCdf& cdf) {
    const auto m = cdf.moments();
    s.n = m.n;
    s.min = m.min;
    s.max = m.max;
    s.mean = m.mean;
    s.stddev = m.stddev;
}

void fill_quantiles(ProbSummary& s, const EmpiricalCdf& cdf) {
    s.p10 = cdf.quantile(0.10);
    s.p50 = cdf.quantile(0.50);
    s.p90 = cdf.quantile(0.90);
    s.p95 = cdf.quantile(0.95);
    s.p99 = cdf.quantile(0.99);
}

void fill_exceed(ProbSummary& s, const EmpiricalCdf& cdf) {
    if (is_finite(s.thr1)) s.p_ge_thr1 = cdf.exceed(s.thr1);
    if (is_finite(s.thr2)) s.p_ge_thr2 = cdf.exceed(s.thr2);
}

} // namespace

ProbSummary summarize(const std::string& case_id,
                      const std::string& metric_name,
                      const EmpiricalCdf& cdf,
                      double thr1,
                      double thr2) {
    ProbSummary s;
    s.case_id = case_id;
    s.metric_name = metric_name;

    s.thr1 = thr1;
    s.thr2 = thr2;

    fill_moments(s, cdf);
    fill_quantiles(s, cdf);
    fill_exceed(s, cdf);

    return s;
}

std::string prob_csv_header() {
    return
        "case_id,metric,n,"
        "min,max,mean,stddev,"
        "p10,p50,p90,p95,p99,"
        "thr1,p_ge_thr1,thr2,p_ge_thr2\n";
}

std::string prob_csv_row(const ProbSummary& s) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(10);

    os
      << esc_csv(s.case_id) << ","
      << esc_csv(s.metric_name) << ","
      << s.n << ","
      << s.min << "," << s.max << "," << s.mean << "," << s.stddev << ","
      << s.p10 << "," << s.p50 << "," << s.p90 << "," << s.p95 << "," << s.p99 << ","
      << s.thr1 << "," << s.p_ge_thr1 << ","
      << s.thr2 << "," << s.p_ge_thr2
      << "\n";

    return os.str();
}

std::string prob_csv(const std::vector<ProbSummary>& ss) {
    std::string out;
    out.reserve(256 + ss.size() * 320);
    out.append(prob_csv_header());
    for (const auto& s : ss) out.append(prob_csv_row(s));
    return out;
}

} // namespace lift::bemt::prob
