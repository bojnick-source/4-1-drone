/*
===============================================================================
Fragment 3.3.01 — Probability CDF Core (Empirical CDF, Quantiles, Exceedance)
File: cdf.cpp
===============================================================================
*/

#include "cdf.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace lift::bemt::prob {

EmpiricalCdf::EmpiricalCdf(const std::vector<double>& samples) {
    reset(samples);
}

void EmpiricalCdf::reset(const std::vector<double>& samples) {
    filter_finite_(samples, xs_);
    sort_(xs_);
}

void EmpiricalCdf::append(const std::vector<double>& samples) {
    if (samples.empty()) return;

    std::vector<double> tmp;
    filter_finite_(samples, tmp);
    if (tmp.empty()) return;

    // Merge in sorted order
    sort_(tmp);
    std::vector<double> merged;
    merged.reserve(xs_.size() + tmp.size());
    std::merge(xs_.begin(), xs_.end(), tmp.begin(), tmp.end(), std::back_inserter(merged));
    xs_.swap(merged);
}

double EmpiricalCdf::cdf(double x) const noexcept {
    if (xs_.empty()) return 0.0;
    if (!is_finite(x)) return 0.0;

    // upper_bound gives first element > x, so count <= x
    const auto it = std::upper_bound(xs_.begin(), xs_.end(), x);
    const double k = static_cast<double>(std::distance(xs_.begin(), it));
    const double n = static_cast<double>(xs_.size());
    const double F = (n > 0.0) ? (k / n) : 0.0;
    return clamp(F, 0.0, 1.0);
}

double EmpiricalCdf::exceed(double t) const noexcept {
    if (xs_.empty()) return 0.0;
    if (!is_finite(t)) return 0.0;

    // P(X >= t) = 1 - P(X < t)
    const auto it = std::lower_bound(xs_.begin(), xs_.end(), t); // first >= t
    const double k_lt = static_cast<double>(std::distance(xs_.begin(), it));
    const double n = static_cast<double>(xs_.size());
    const double p = (n > 0.0) ? (1.0 - (k_lt / n)) : 0.0;
    return clamp(p, 0.0, 1.0);
}

double EmpiricalCdf::quantile(double p) const noexcept {
    if (xs_.empty()) return 0.0;
    if (!is_finite(p)) return 0.0;

    const double pp = clamp(p, 0.0, 1.0);
    const std::size_t n = xs_.size();
    if (n == 1) return xs_[0];

    // R type=7
    const double h = 1.0 + (static_cast<double>(n) - 1.0) * pp;
    const double hf = std::floor(h);
    const std::size_t j = static_cast<std::size_t>(std::max(1.0, std::min(hf, static_cast<double>(n)))) - 1; // 0-based
    const double g = h - hf;

    if (j + 1 >= n) return xs_.back();
    const double xj = xs_[j];
    const double xk = xs_[j + 1];
    const double q = (1.0 - g) * xj + g * xk;
    return is_finite(q) ? q : 0.0;
}

Moments EmpiricalCdf::moments() const noexcept {
    Moments m;
    m.n = xs_.size();
    if (m.n == 0) return m;

    m.min = xs_.front();
    m.max = xs_.back();

    // Welford for mean/variance (population)
    double mean = 0.0;
    double M2 = 0.0;
    std::size_t k = 0;

    for (double x : xs_) {
        ++k;
        const double delta = x - mean;
        mean += delta / static_cast<double>(k);
        const double delta2 = x - mean;
        M2 += delta * delta2;
    }

    m.mean = mean;
    m.variance = (m.n > 0) ? (M2 / static_cast<double>(m.n)) : 0.0;
    m.stddev = std::sqrt(std::max(0.0, m.variance));

    if (!is_finite(m.mean)) m.mean = 0.0;
    if (!is_finite(m.variance)) m.variance = 0.0;
    if (!is_finite(m.stddev)) m.stddev = 0.0;

    return m;
}

void EmpiricalCdf::filter_finite_(const std::vector<double>& in, std::vector<double>& out) {
    out.clear();
    out.reserve(in.size());
    for (double x : in) {
        if (is_finite(x)) out.push_back(x);
    }
}

void EmpiricalCdf::sort_(std::vector<double>& v) {
    std::sort(v.begin(), v.end());
}

} // namespace lift::bemt::prob
