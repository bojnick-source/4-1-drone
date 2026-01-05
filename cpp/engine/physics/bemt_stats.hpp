#pragma once

#include "engine/physics/bemt.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

namespace lift::bemt {

// --------------------------------------------
// OnlineStats: hardened Welford accumulator
// --------------------------------------------
struct OnlineStats final {
    std::uint64_t n = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double minv = +std::numeric_limits<double>::infinity();
    double maxv = -std::numeric_limits<double>::infinity();

    void reset() noexcept {
        n = 0; mean = 0.0; m2 = 0.0;
        minv = +std::numeric_limits<double>::infinity();
        maxv = -std::numeric_limits<double>::infinity();
    }

    void push(double x) noexcept {
        if (!is_finite(x)) return; // ignore non-finite values (never poison stats)
        ++n;
        if (x < minv) minv = x;
        if (x > maxv) maxv = x;

        const double delta = x - mean;
        mean += delta / static_cast<double>(n);
        const double delta2 = x - mean;
        m2 += delta * delta2;
    }

    double variance() const noexcept {
        if (n < 2) return 0.0;
        const double v = m2 / static_cast<double>(n - 1);
        return is_finite(v) && v >= 0.0 ? v : 0.0;
    }

    double stddev() const noexcept {
        const double v = variance();
        return v > 0.0 ? std::sqrt(v) : 0.0;
    }

    bool valid() const noexcept { return n > 0; }
};

// --------------------------------------------
// Empirical CDF: deterministic sample CDF
// --------------------------------------------
struct EmpiricalCDF final {
    std::vector<double> samples;   // unsorted until finalize()
    bool finalized = false;

    void clear() {
        samples.clear();
        finalized = false;
    }

    void reserve(std::size_t n) { samples.reserve(n); }

    void push(double x) {
        if (!is_finite(x)) return;
        samples.push_back(x);
        finalized = false;
    }

    void finalize() {
        // Remove non-finite defensively (should already be filtered)
        samples.erase(std::remove_if(samples.begin(), samples.end(),
                                     [](double v) { return !is_finite(v); }),
                      samples.end());
        std::sort(samples.begin(), samples.end());
        finalized = true;
    }

    std::size_t size() const noexcept { return samples.size(); }

    // CDF(x) = P(X <= x) in [0,1]
    double cdf(double x) const noexcept {
        if (!finalized || samples.empty() || !is_finite(x)) return 0.0;
        auto it = std::upper_bound(samples.begin(), samples.end(), x);
        const std::size_t k = static_cast<std::size_t>(std::distance(samples.begin(), it));
        return static_cast<double>(k) / static_cast<double>(samples.size());
    }

    // Quantile(p) with p in [0,1], using nearest-rank with clamping
    double quantile(double p) const noexcept {
        if (!finalized || samples.empty() || !is_finite(p)) return 0.0;
        p = clamp(p, 0.0, 1.0);
        if (samples.size() == 1) return samples[0];

        const double idx = p * static_cast<double>(samples.size() - 1);
        const std::size_t i0 = static_cast<std::size_t>(idx);
        const std::size_t i1 = std::min(i0 + 1, samples.size() - 1);
        const double t = idx - static_cast<double>(i0);

        const double v = samples[i0] + t * (samples[i1] - samples[i0]);
        return is_finite(v) ? v : samples[i0];
    }
};

// --------------------------------------------
// Station metric selection (for CDF hooks)
// --------------------------------------------
enum class StationMetric : std::uint8_t {
    AoA_rad = 0,
    Phi_rad,
    Cl,
    Cd,
    dT_N,
    dQ_Nm,
    Vax_mps,
    Vtan_mps,
    Vrel_mps,
    Reynolds,
    Mach,
    TipLossF
};

inline double station_metric_value(const StationResult& s, StationMetric m) noexcept {
    switch (m) {
        case StationMetric::AoA_rad:   return s.aoa_rad;
        case StationMetric::Phi_rad:   return s.phi_rad;
        case StationMetric::Cl:        return s.cl;
        case StationMetric::Cd:        return s.cd;
        case StationMetric::dT_N:      return s.dT_N;
        case StationMetric::dQ_Nm:     return s.dQ_Nm;
        case StationMetric::Vax_mps:   return s.V_axial_m_s;
        case StationMetric::Vtan_mps:  return s.V_tan_m_s;
        case StationMetric::Vrel_mps:  return s.V_rel_m_s;
        case StationMetric::Reynolds:  return s.reynolds;
        case StationMetric::Mach:      return s.mach;
        case StationMetric::TipLossF:  return s.tip_loss_F;
        default:                       return 0.0;
    }
}

inline const char* station_metric_name(StationMetric m) noexcept {
    switch (m) {
        case StationMetric::AoA_rad:  return "aoa_rad";
        case StationMetric::Phi_rad:  return "phi_rad";
        case StationMetric::Cl:       return "cl";
        case StationMetric::Cd:       return "cd";
        case StationMetric::dT_N:     return "dT_N";
        case StationMetric::dQ_Nm:    return "dQ_Nm";
        case StationMetric::Vax_mps:  return "Vax_mps";
        case StationMetric::Vtan_mps: return "Vtan_mps";
        case StationMetric::Vrel_mps: return "Vrel_mps";
        case StationMetric::Reynolds: return "Re";
        case StationMetric::Mach:     return "Mach";
        case StationMetric::TipLossF: return "TipLossF";
        default:                      return "unknown";
    }
}

// --------------------------------------------
// Deterministic stats bundle from one BemtResult
// --------------------------------------------
struct BemtStationStats final {
    StationMetric metric = StationMetric::AoA_rad;
    OnlineStats stats;
    EmpiricalCDF cdf; // optional; finalize() must be called by user if they want quantiles

    void clear(StationMetric m) {
        metric = m;
        stats.reset();
        cdf.clear();
    }

    void ingest(const BemtResult& r, bool collect_cdf_samples) {
        for (const auto& s : r.stations) {
            const double v = station_metric_value(s, metric);
            stats.push(v);
            if (collect_cdf_samples) cdf.push(v);
        }
    }
};

// --------------------------------------------
// Multi-run aggregator (optimization loop hook)
// --------------------------------------------
struct AggregateStats final {
    // Totals across runs
    OnlineStats thrust_N;
    OnlineStats power_W;
    OnlineStats torque_Nm;
    OnlineStats induced_mps;

    // Optional per-station metric samples across runs
    StationMetric station_metric = StationMetric::AoA_rad;
    OnlineStats station_metric_stats;
    EmpiricalCDF station_metric_cdf; // optional

    void reset(StationMetric m = StationMetric::AoA_rad) {
        thrust_N.reset();
        power_W.reset();
        torque_Nm.reset();
        induced_mps.reset();
        station_metric = m;
        station_metric_stats.reset();
        station_metric_cdf.clear();
    }

    void ingest_run(const BemtResult& r, bool collect_station_cdf_samples) {
        thrust_N.push(r.thrust_N);
        power_W.push(r.power_W);
        torque_Nm.push(r.torque_Nm);
        induced_mps.push(r.induced_velocity_m_s);

        for (const auto& s : r.stations) {
            const double v = station_metric_value(s, station_metric);
            station_metric_stats.push(v);
            if (collect_station_cdf_samples) station_metric_cdf.push(v);
        }
    }
};

} // namespace lift::bemt

