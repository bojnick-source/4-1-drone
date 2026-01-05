// ============================================================================
// Fragment 3.3.01 — Probability CDF Core (Empirical CDF, Quantiles, Exceedance)
// File: cdf.hpp
// ============================================================================
//
// Purpose:
// - Minimal, hardened probability utilities for stats reporting (CDF as requested).
// - Empirical CDF from samples (after filtering non-finite values).
// - Quantiles, exceedance probability, and basic moments.
//
// Notes:
// - Deterministic: stable sorting, stable quantile definition.
// - Zero dependencies beyond STL.
// - Designed for optimizer telemetry + closeout probability reporting.
//
// Quantile definition (type=7 like R default):
//   q(p) = (1-g)*x[j] + g*x[j+1]
//   where h = 1 + (n-1)*p, j = floor(h), g = h-j
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::bemt::prob {

struct Moments final {
    std::size_t n = 0;
    double min = 0.0;
    double max = 0.0;
    double mean = 0.0;
    double variance = 0.0; // population variance (divide by n)
    double stddev = 0.0;

    bool valid() const noexcept { return n > 0 && is_finite(mean) && is_finite(stddev); }
};

// Empirical CDF container (stores sorted samples).
class EmpiricalCdf final {
public:
    EmpiricalCdf() = default;

    // Construct from raw samples (filters non-finite).
    explicit EmpiricalCdf(const std::vector<double>& samples);

    // Replace data (filters non-finite).
    void reset(const std::vector<double>& samples);

    // Append samples (filters non-finite) and re-sort.
    void append(const std::vector<double>& samples);

    // Accessors
    std::size_t size() const noexcept { return xs_.size(); }
    bool empty() const noexcept { return xs_.empty(); }
    const std::vector<double>& sorted() const noexcept { return xs_; }

    // CDF F(x) = P(X <= x)
    // Returns 0 if empty.
    double cdf(double x) const noexcept;

    // Survival S(x) = P(X > x)
    double survival(double x) const noexcept { return 1.0 - cdf(x); }

    // Exceedance probability for threshold t:
    // P(X >= t) using right-tail with inclusive threshold.
    // Implemented as 1 - P(X < t).
    double exceed(double t) const noexcept;

    // Quantile q(p) for p in [0,1].
    // Returns 0 if empty.
    double quantile(double p) const noexcept;

    // Median convenience.
    double median() const noexcept { return quantile(0.5); }

    // Basic moments.
    Moments moments() const noexcept;

private:
    static void filter_finite_(const std::vector<double>& in, std::vector<double>& out);
    static void sort_(std::vector<double>& v);

private:
    std::vector<double> xs_;
};

} // namespace lift::bemt::prob
