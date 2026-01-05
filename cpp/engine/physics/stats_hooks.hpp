/*
===============================================================================
Fragment 3.1.14 — Stats Hooks (Running Moments + Empirical CDF/Quantiles) (C++)
File: stats_hooks.hpp
===============================================================================
*/

#pragma once
#include "bemt_types.hpp"
#include "bemt_require.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace lift::bemt::stats {

// Online stats (Welford) with hard finite guards.
// Keeps mean/variance stable for large N.
struct RunningStats final {
    std::size_t n = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double minv = std::numeric_limits<double>::infinity();
    double maxv = -std::numeric_limits<double>::infinity();

    void reset() noexcept {
        n = 0;
        mean = 0.0;
        m2 = 0.0;
        minv = std::numeric_limits<double>::infinity();
        maxv = -std::numeric_limits<double>::infinity();
    }

    void push(double x) noexcept {
        if (!is_finite(x)) return;

        ++n;
        const double delta = x - mean;
        mean += delta / static_cast<double>(n);
        const double delta2 = x - mean;
        m2 += delta * delta2;

        minv = std::min(minv, x);
        maxv = std::max(maxv, x);
    }

    double variance() const noexcept {
        if (n < 2) return 0.0;
        const double v = m2 / static_cast<double>(n - 1);
        return (is_finite(v) && v >= 0.0) ? v : 0.0;
    }

    double stddev() const noexcept {
        const double v = variance();
        const double s = std::sqrt(std::max(0.0, v));
        return is_finite(s) ? s : 0.0;
    }
};

// Reservoir sampler for empirical CDF + quantiles with bounded memory.
// - Stores up to cap samples uniformly over stream.
class Reservoir final {
public:
    explicit Reservoir(std::size_t cap = 8192, std::uint64_t seed = 0xC0FFEEULL)
        : cap_(cap), rng_(static_cast<std::mt19937_64::result_type>(seed)) {
        if (cap_ == 0) cap_ = 1;
        data_.reserve(cap_);
    }

    void reset(std::uint64_t seed) {
        rng_.seed(static_cast<std::mt19937_64::result_type>(seed));
        data_.clear();
        seen_ = 0;
        sorted_ = false;
    }

    void push(double x) {
        if (!is_finite(x)) return;
        ++seen_;

        if (data_.size() < cap_) {
            data_.push_back(x);
            sorted_ = false;
            return;
        }

        // Reservoir replacement with probability cap/seen.
        std::uniform_int_distribution<std::uint64_t> U(0ULL, seen_ - 1ULL);
        const std::uint64_t j = U(rng_);
        if (j < cap_) {
            data_[static_cast<std::size_t>(j)] = x;
            sorted_ = false;
        }
    }

    std::size_t seen() const noexcept { return seen_; }
    std::size_t size() const noexcept { return data_.size(); }
    std::size_t cap()  const noexcept { return cap_; }

    // Ensure sorted view for quantiles/CDF.
    void sort_if_needed() {
        if (sorted_) return;
        std::sort(data_.begin(), data_.end());
        sorted_ = true;
    }

    // Empirical CDF at x: P(X <= x)
    double cdf(double x) {
        if (data_.empty()) return 0.0;
        sort_if_needed();
        const auto it = std::upper_bound(data_.begin(), data_.end(), x);
        const double k = static_cast<double>(std::distance(data_.begin(), it));
        return clamp(k / static_cast<double>(data_.size()), 0.0, 1.0);
    }

    // Quantile q in [0,1], linear interpolation between adjacent order stats.
    double quantile(double q) {
        if (data_.empty()) return 0.0;
        sort_if_needed();
        q = clamp(q, 0.0, 1.0);
        const double idx = q * static_cast<double>(data_.size() - 1);
        const std::size_t i0 = static_cast<std::size_t>(std::floor(idx));
        const std::size_t i1 = std::min(i0 + 1, data_.size() - 1);
        const double t = idx - static_cast<double>(i0);
        return data_[i0] + (data_[i1] - data_[i0]) * t;
    }

private:
    std::size_t cap_ = 8192;
    std::mt19937_64 rng_;
    std::vector<double> data_;
    std::uint64_t seen_ = 0;
    bool sorted_ = false;
};

// One metric bundle: running moments + bounded reservoir for CDF/quantiles.
struct MetricStats final {
    RunningStats moments;
    Reservoir reservoir;

    explicit MetricStats(std::size_t cap = 8192, std::uint64_t seed = 0xC0FFEEULL)
        : reservoir(cap, seed) {}

    void reset(std::uint64_t seed) {
        moments.reset();
        reservoir.reset(seed);
    }

    void push(double x) noexcept {
        moments.push(x);
        reservoir.push(x);
    }

    double mean() const noexcept { return moments.mean; }
    double stddev() const noexcept { return moments.stddev(); }
    double minv() const noexcept { return (is_finite(moments.minv) ? moments.minv : 0.0); }
    double maxv() const noexcept { return (is_finite(moments.maxv) ? moments.maxv : 0.0); }
};

// Full uncertainty report for BEMT outputs.
// Add fields as closeout expands; keep names stable for downstream.
struct UncertaintyReport final {
    std::uint64_t seed = 0;
    std::size_t cap = 8192;

    MetricStats thrust_N;
    MetricStats power_W;
    MetricStats torque_Nm;
    MetricStats vi_mps;
    MetricStats fm;
    MetricStats collective_rad;

    explicit UncertaintyReport(std::size_t cap_in = 8192, std::uint64_t seed_in = 0xC0FFEEULL)
        : seed(seed_in), cap(cap_in),
          thrust_N(cap_in, seed_in ^ 0x01ULL),
          power_W(cap_in, seed_in ^ 0x02ULL),
          torque_Nm(cap_in, seed_in ^ 0x03ULL),
          vi_mps(cap_in, seed_in ^ 0x04ULL),
          fm(cap_in, seed_in ^ 0x05ULL),
          collective_rad(cap_in, seed_in ^ 0x06ULL) {}

    void reset() {
        thrust_N.reset(seed ^ 0x01ULL);
        power_W.reset(seed ^ 0x02ULL);
        torque_Nm.reset(seed ^ 0x03ULL);
        vi_mps.reset(seed ^ 0x04ULL);
        fm.reset(seed ^ 0x05ULL);
        collective_rad.reset(seed ^ 0x06ULL);
    }

    void push_sample(const BemtResult& r) {
        thrust_N.push(r.thrust_N);
        power_W.push(r.power_W);
        torque_Nm.push(r.torque_Nm);
        vi_mps.push(r.induced_velocity_m_s);
        fm.push(r.figure_of_merit);
        collective_rad.push(r.collective_offset_rad);
    }
};

} // namespace lift::bemt::stats
