// ============================================================================
// Fragment 3.1.16 — Uncertainty/Stats Hooks (Online Mean/Var + Min/Max + Empirical CDF Builder, Hardened) (C++)
// File: online_stats.hpp
// ============================================================================
//
// Purpose:
// - Provide a fast, allocation-light stats hook for BEMT + optimizer closeout.
// - Online (single-pass) statistics for streaming samples:
//     count, mean, variance (Welford), stddev, min/max
// - Optional sample reservoir (bounded) to enable empirical quantiles/CDF later.
// - Deterministic behavior with explicit caps and NaN filtering.
//
// Policy alignment:
// - “CDF” here means cumulative distribution functions for probability/stat reporting.
// - This file provides the infrastructure; you can choose when to store samples.
// - In hot loops: keep reservoir disabled or small.
// - In top-N closeout / selective validation: enable reservoir for quantiles/CDF.
//
// ============================================================================

#pragma once
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace lift::stats {

// -----------------------------
// OnlineStats (Welford)
// -----------------------------
struct OnlineStats final {
    std::uint64_t n = 0;
    double mean = 0.0;
    double M2 = 0.0; // sum of squares of differences from the current mean
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = -std::numeric_limits<double>::infinity();

    // Optionally track sum for sanity (not needed for mean)
    double sum = 0.0;

    void reset() noexcept {
        n = 0;
        mean = 0.0;
        M2 = 0.0;
        min_v = std::numeric_limits<double>::infinity();
        max_v = -std::numeric_limits<double>::infinity();
        sum = 0.0;
    }

    void push(double x) noexcept {
        if (!lift::bemt::is_finite(x)) return;

        ++n;
        sum += x;

        if (x < min_v) min_v = x;
        if (x > max_v) max_v = x;

        // Welford update
        const double delta = x - mean;
        mean += delta / static_cast<double>(n);
        const double delta2 = x - mean;
        M2 += delta * delta2;

        if (!lift::bemt::is_finite(mean)) { mean = 0.0; M2 = 0.0; } // hard recovery
        if (!lift::bemt::is_finite(M2) || M2 < 0.0) M2 = 0.0;
    }

    std::uint64_t count() const noexcept { return n; }

    double variance_population() const noexcept {
        if (n == 0) return 0.0;
        const double v = M2 / static_cast<double>(n);
        return (lift::bemt::is_finite(v) && v >= 0.0) ? v : 0.0;
    }

    double variance_sample() const noexcept {
        if (n < 2) return 0.0;
        const double v = M2 / static_cast<double>(n - 1);
        return (lift::bemt::is_finite(v) && v >= 0.0) ? v : 0.0;
    }

    double stddev_population() const noexcept {
        return lift::bemt::safe_sqrt(variance_population(), 0.0);
    }

    double stddev_sample() const noexcept {
        return lift::bemt::safe_sqrt(variance_sample(), 0.0);
    }

    double min() const noexcept {
        if (n == 0) return 0.0;
        return lift::bemt::is_finite(min_v) ? min_v : 0.0;
    }

    double max() const noexcept {
        if (n == 0) return 0.0;
        return lift::bemt::is_finite(max_v) ? max_v : 0.0;
    }
};

// -----------------------------
// Reservoir (bounded sample storage)
// -----------------------------
struct ReservoirConfig final {
    std::size_t max_samples = 0;   // 0 => disabled
    bool store_finite_only = true; // recommended
    bool store_clamped = false;    // if false, caller decides clamping; this just stores
    void validate() const {
        LIFT_BEMT_REQUIRE(max_samples <= 5'000'000, lift::bemt::ErrorCode::InvalidConfig, "ReservoirConfig.max_samples too large");
    }
};

struct Reservoir final {
    ReservoirConfig cfg{};
    std::vector<double> samples;

    explicit Reservoir(const ReservoirConfig& c = {}) : cfg(c) {
        cfg.validate();
        if (cfg.max_samples > 0) samples.reserve(std::min<std::size_t>(cfg.max_samples, 4096));
    }

    void reset() {
        samples.clear();
    }

    std::size_t capacity() const noexcept { return cfg.max_samples; }
    std::size_t size() const noexcept { return samples.size(); }
    bool enabled() const noexcept { return cfg.max_samples > 0; }

    void push(double x) {
        if (!enabled()) return;
        if (cfg.store_finite_only && !lift::bemt::is_finite(x)) return;
        if (samples.size() < cfg.max_samples) samples.push_back(x);
        // If full: deterministic drop policy (drop newest). (No randomness in hot loops.)
    }
};

// -----------------------------
// Combined accumulator for “hooks”
// -----------------------------
struct StatsHook final {
    OnlineStats online{};
    Reservoir reservoir{};

    explicit StatsHook(const ReservoirConfig& rc = {}) : reservoir(rc) {}

    void reset() {
        online.reset();
        reservoir.reset();
    }

    void push(double x) {
        online.push(x);
        reservoir.push(x);
    }
};

// -----------------------------
// Empirical CDF builder (from stored samples)
// -----------------------------
// Returns pairs (x_i, F_i) for i in [0..k-1], where F_i = (i+1)/k.
struct EmpiricalCdf final {
    std::vector<double> x;
    std::vector<double> F;

    bool ok() const noexcept { return x.size() == F.size() && !x.empty(); }
};

struct CdfBuildConfig final {
    bool sort_samples = true; // always true for valid empirical CDF
    bool unique_x = false;    // if true, compress identical x into steps
    std::size_t max_points = 0; // 0 => no downsample, else downsample to <= max_points
    void validate() const {
        LIFT_BEMT_REQUIRE(max_points <= 2'000'000, lift::bemt::ErrorCode::InvalidConfig, "CdfBuildConfig.max_points too large");
    }
};

// Build an empirical CDF from a reservoir (copies then sorts by default).
inline EmpiricalCdf build_empirical_cdf(const Reservoir& r, const CdfBuildConfig& cfg_in = {}) {
    CdfBuildConfig cfg = cfg_in;
    cfg.validate();

    EmpiricalCdf out;
    if (!r.enabled() || r.samples.empty()) return out;

    out.x = r.samples;
    if (cfg.sort_samples) std::sort(out.x.begin(), out.x.end());

    const std::size_t n = out.x.size();
    out.F.reserve(n);

    if (!cfg.unique_x) {
        for (std::size_t i = 0; i < n; ++i) {
            const double Fi = static_cast<double>(i + 1) / static_cast<double>(n);
            out.F.push_back(Fi);
        }
    } else {
        // Compress identical x values into the last step probability.
        std::vector<double> x2;
        std::vector<double> F2;
        x2.reserve(n);
        F2.reserve(n);

        std::size_t i = 0;
        while (i < n) {
            const double xi = out.x[i];
            std::size_t j = i + 1;
            while (j < n && out.x[j] == xi) ++j;

            const double Fi = static_cast<double>(j) / static_cast<double>(n);
            x2.push_back(xi);
            F2.push_back(Fi);
            i = j;
        }

        out.x.swap(x2);
        out.F.swap(F2);
    }

    // Optional downsample (uniform stride), deterministic
    if (cfg.max_points > 0 && out.x.size() > cfg.max_points) {
        const std::size_t m = cfg.max_points;
        std::vector<double> xd; xd.reserve(m);
        std::vector<double> Fd; Fd.reserve(m);

        const double step = static_cast<double>(out.x.size() - 1) / static_cast<double>(m - 1);
        for (std::size_t k = 0; k < m; ++k) {
            const std::size_t idx = static_cast<std::size_t>(std::llround(step * static_cast<double>(k)));
            xd.push_back(out.x[std::min(idx, out.x.size() - 1)]);
            Fd.push_back(out.F[std::min(idx, out.F.size() - 1)]);
        }

        out.x.swap(xd);
        out.F.swap(Fd);
    }

    return out;
}

// -----------------------------
// Convenience: common summary row for CSV
// -----------------------------
struct SummaryRow final {
    std::uint64_t n = 0;
    double mean = 0.0;
    double std_sample = 0.0;
    double min_v = 0.0;
    double max_v = 0.0;
};

inline SummaryRow summarize(const OnlineStats& s) noexcept {
    SummaryRow r;
    r.n = s.count();
    r.mean = lift::bemt::is_finite(s.mean) ? s.mean : 0.0;
    r.std_sample = s.stddev_sample();
    r.min_v = s.min();
    r.max_v = s.max();
    return r;
}

} // namespace lift::stats
