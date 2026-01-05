/*
===============================================================================
Fragment 3.1.45 — Statistical Reporting Hooks (Empirical CDF, Quantiles, Probability of Constraint Violation) (C++)
File: cpp/engine/stats/empirical_cdf.hpp
===============================================================================
*/

#pragma once
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace lift::stats {

struct Summary final {
    std::size_t n = 0;
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    double stdev = 0.0;      // sample stdev (n>1 uses n-1 denominator)

    void clear() noexcept { *this = Summary{}; }
};

inline double clamp01(double p) noexcept {
    if (!lift::bemt::is_finite(p)) return 0.0;
    if (p < 0.0) return 0.0;
    if (p > 1.0) return 1.0;
    return p;
}

inline bool is_valid_sample(double x) noexcept {
    return lift::bemt::is_finite(x);
}

// -----------------------------
// Empirical CDF (ECDF)
// -----------------------------
class EmpiricalCDF final {
public:
    EmpiricalCDF() = default;

    void reserve(std::size_t n) { samples_.reserve(n); }

    void clear() noexcept {
        samples_.clear();
        sorted_ = false;
        summary_.clear();
    }

    std::size_t size() const noexcept { return samples_.size(); }
    bool empty() const noexcept { return samples_.empty(); }
    const Summary& summary() const noexcept { return summary_; }

    // Adds a sample. NaN/Inf are ignored (by design).
    void push(double x) {
        if (!is_valid_sample(x)) return;
        samples_.push_back(x);
        sorted_ = false;
    }

    // Bulk add
    void extend(const std::vector<double>& xs) {
        for (double x : xs) push(x);
    }

    // Finalize:
    // - sorts samples
    // - computes summary stats
    // Safe to call multiple times; idempotent if no new samples.
    void finalize() {
        if (samples_.empty()) {
            sorted_ = true;
            summary_.clear();
            return;
        }
        if (!sorted_) {
            std::sort(samples_.begin(), samples_.end());
            sorted_ = true;
        }
        compute_summary_();
    }

    // Empirical CDF: P(X <= x)
    // - If no samples -> 0
    // - Requires finalize() for best performance; will finalize lazily if needed.
    double cdf(double x) {
        if (samples_.empty()) return 0.0;
        ensure_sorted_();
        // upper_bound gives iterator to first element > x
        auto it = std::upper_bound(samples_.begin(), samples_.end(), x);
        const std::size_t k = static_cast<std::size_t>(std::distance(samples_.begin(), it));
        return static_cast<double>(k) / static_cast<double>(samples_.size());
    }

    // Complementary CDF: P(X > x)
    double ccdf(double x) { return 1.0 - cdf(x); }

    // Quantile:
    // - p in [0,1]
    // - Uses nearest-rank with linear interpolation between adjacent points.
    // - If n==1 returns that value.
    double quantile(double p) {
        if (samples_.empty()) return 0.0;
        ensure_sorted_();
        const double pp = clamp01(p);
        const std::size_t n = samples_.size();
        if (n == 1) return samples_[0];

        // position in [0, n-1]
        const double pos = pp * static_cast<double>(n - 1);
        const std::size_t i0 = static_cast<std::size_t>(std::floor(pos));
        const std::size_t i1 = std::min(n - 1, i0 + 1);
        const double t = pos - static_cast<double>(i0);
        const double a = samples_[i0];
        const double b = samples_[i1];
        const double q = a + t * (b - a);
        return (lift::bemt::is_finite(q) ? q : a);
    }

    // Convenience percentiles
    double p50() { return quantile(0.50); }
    double p90() { return quantile(0.90); }
    double p95() { return quantile(0.95); }
    double p99() { return quantile(0.99); }

    // Probability gates (risk):
    // P(X <= thr), P(X >= thr), P(X > thr), P(X < thr)
    double prob_leq(double thr) { return cdf(thr); }
    double prob_gt(double thr)  { return ccdf(thr); }

    double prob_geq(double thr) {
        // P(X >= thr) = 1 - P(X < thr) = 1 - cdf(thr - eps)
        // Use nextafter to avoid floating exactness issues.
        const double thr_minus = std::nextafter(thr, -std::numeric_limits<double>::infinity());
        return 1.0 - cdf(thr_minus);
    }

    double prob_lt(double thr) {
        const double thr_minus = std::nextafter(thr, -std::numeric_limits<double>::infinity());
        return cdf(thr_minus);
    }

    // Access raw sorted samples (forces finalize)
    const std::vector<double>& sorted_samples() {
        ensure_sorted_();
        return samples_;
    }

private:
    void ensure_sorted_() {
        if (!sorted_) finalize();
        else if (summary_.n != samples_.size()) compute_summary_();
    }

    void compute_summary_() {
        Summary s;
        s.n = samples_.size();
        if (s.n == 0) { summary_ = s; return; }

        s.min = samples_.front();
        s.max = samples_.back();

        // Welford (stable) with sample variance (n-1)
        double mean = 0.0;
        double M2 = 0.0;
        std::size_t k = 0;
        for (double x : samples_) {
            ++k;
            const double delta = x - mean;
            mean += delta / static_cast<double>(k);
            const double delta2 = x - mean;
            M2 += delta * delta2;
        }
        s.mean = lift::bemt::is_finite(mean) ? mean : 0.0;
        const double denom = (k > 1) ? static_cast<double>(k - 1) : 1.0;
        const double var = (k > 1) ? (M2 / denom) : 0.0;
        s.stdev = std::sqrt(var);
        if (!lift::bemt::is_finite(s.stdev)) s.stdev = 0.0;

        summary_ = s;
    }

    std::vector<double> samples_;
    bool sorted_ = false;
    Summary summary_{};
};

// -----------------------------
// Multi-metric risk packaging
// -----------------------------
struct RiskItem final {
    std::string metric_id;   // e.g., "P_HOVER_1G_W"
    std::string comparator;  // "<=", ">=", "<", ">"
    double threshold = 0.0;

    // computed
    double probability = 0.0; // probability of satisfying comparator (pass probability)
    double fail_probability = 0.0;

    double p50 = 0.0;
    double p90 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;

    Summary summary{};
};

inline bool compare(double lhs, const std::string& cmp, double rhs) noexcept {
    if (cmp == "<=") return lhs <= rhs;
    if (cmp == "<")  return lhs <  rhs;
    if (cmp == ">=") return lhs >= rhs;
    if (cmp == ">")  return lhs >  rhs;
    return false;
}

// Given an ECDF, compute pass probability against a threshold/comparator.
inline std::pair<double,double> pass_fail_probability(EmpiricalCDF& ecdf, const std::string& cmp, double thr) {
    ecdf.finalize();
    if (ecdf.empty()) return {0.0, 0.0};

    double p_pass = 0.0;
    if (cmp == "<=") p_pass = ecdf.prob_leq(thr);
    else if (cmp == "<") p_pass = ecdf.prob_lt(thr);
    else if (cmp == ">=") p_pass = ecdf.prob_geq(thr);
    else if (cmp == ">") p_pass = ecdf.prob_gt(thr);
    else p_pass = 0.0;

    p_pass = clamp01(p_pass);
    const double p_fail = clamp01(1.0 - p_pass);
    return {p_pass, p_fail};
}

// Build a RiskItem for a metric distribution
inline RiskItem build_risk_item(const std::string& metric_id,
                                const std::string& comparator,
                                double threshold,
                                EmpiricalCDF& ecdf) {
    RiskItem r;
    r.metric_id = metric_id;
    r.comparator = comparator;
    r.threshold = threshold;

    ecdf.finalize();
    r.summary = ecdf.summary();

    r.p50 = ecdf.p50();
    r.p90 = ecdf.p90();
    r.p95 = ecdf.p95();
    r.p99 = ecdf.p99();

    auto [pp, pf] = pass_fail_probability(ecdf, comparator, threshold);
    r.probability = pp;
    r.fail_probability = pf;

    return r;
}

} // namespace lift::stats
