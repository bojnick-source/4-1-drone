#pragma once

#include "engine/physics/bemt_solver.hpp"
#include "engine/physics/bemt_polar.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <limits>
#include <utility>
#include <vector>
#include <cmath>

namespace lift::bemt {

// This layer isolates BEMT from:
// - CAD/mesh geometry (OpenCASCADE/CGAL/PicoGK)
// - airfoil table databases (CSV/HDF5/custom)
// - optimization parameterizations (GA/PSO/NLopt)
//
// The goal: BEMT core stays stable while upstream systems change.

// ------------------------------
// Rotor parameterization
// ------------------------------
struct RotorParam final {
    // High-level rotor identifiers (for caching & traceability)
    std::string rotor_id;

    // Geometry essentials
    std::size_t blade_count = 0;
    double radius_m = 0.0;
    double hub_radius_m = 0.0;

    TipLossModel tip_loss = TipLossModel::Prandtl;

    // Station definition by normalized radius r/R
    std::vector<double> r_over_R;    // strictly increasing, in (hub/R, 1]
    std::vector<double> chord_m;     // same size
    std::vector<double> twist_rad;   // same size
    std::vector<std::string> airfoil_id; // same size; defines which airfoil polar to use at that station

    void validate() const {
        if (blade_count < 2 || blade_count > 128) {
            throw BemtError(ErrorCode::InvalidInput, "RotorParam.blade_count invalid");
        }
        if (!is_finite(radius_m) || radius_m <= 0.0 || radius_m > 100.0) {
            throw BemtError(ErrorCode::InvalidInput, "RotorParam.radius invalid");
        }
        if (!is_finite(hub_radius_m) || hub_radius_m < 0.0 || hub_radius_m >= radius_m) {
            throw BemtError(ErrorCode::InvalidInput, "RotorParam.hub_radius invalid");
        }
        const std::size_t n = r_over_R.size();
        if (n < 5 || chord_m.size() != n || twist_rad.size() != n || airfoil_id.size() != n) {
            throw BemtError(ErrorCode::InvalidInput, "RotorParam station arrays size mismatch or too small");
        }
        double prev = -1e9;
        for (std::size_t i = 0; i < n; ++i) {
            const double rr = r_over_R[i];
            if (!is_finite(rr) || rr <= hub_radius_m / radius_m || rr > 1.0) {
                throw BemtError(ErrorCode::InvalidInput, "RotorParam.r_over_R out of bounds");
            }
            if (rr <= prev) {
                throw BemtError(ErrorCode::InvalidInput, "RotorParam.r_over_R not strictly increasing");
            }
            prev = rr;
            if (!is_finite(chord_m[i]) || chord_m[i] <= 0.0) {
                throw BemtError(ErrorCode::InvalidInput, "RotorParam.chord invalid");
            }
            if (!is_finite(twist_rad[i])) {
                throw BemtError(ErrorCode::InvalidInput, "RotorParam.twist invalid");
            }
            if (airfoil_id[i].empty()) {
                throw BemtError(ErrorCode::InvalidInput, "RotorParam.airfoil_id empty");
            }
        }
    }
};

// ------------------------------
// Airfoil table interface
// ------------------------------
struct PolarRequest final {
    std::string airfoil_id;
    // Desired nominal Re/Mach slices (table can decide best available)
    double reynolds = 1.0e6;
    double mach = 0.1;
};

class IAirfoilDatabase {
public:
    virtual ~IAirfoilDatabase() = default;

    // Return an immutable polar object for a given airfoil.
    // Implementations should internally cache to avoid re-parsing tables.
    virtual std::shared_ptr<const IAirfoilPolar> get_polar(const PolarRequest& req) const = 0;
};

// ------------------------------
// Geometry builder: creates RotorGeometry from paramization
// ------------------------------
struct GeometryBuildOptions final {
    // If true, chord/twist arrays are resampled to N stations (uniform in r).
    // If false, uses input station arrays directly.
    bool resample = false;
    std::size_t resample_N = 25;

    void validate() const {
        if (resample) {
            if (resample_N < 9 || resample_N > 201)
                throw BemtError(ErrorCode::InvalidInput, "GeometryBuildOptions.resample_N out of range [9..201]");
        }
    }
};

// Build a RotorGeometry using the provided RotorParam arrays.
// The produced RotorGeometry stores only r, chord, twist (airfoil mapping handled elsewhere).
inline RotorGeometry build_rotor_geometry(const RotorParam& p, const GeometryBuildOptions& opt) {
    p.validate();
    opt.validate();

    RotorGeometry g;
    g.blade_count = p.blade_count;
    g.radius_m = p.radius_m;
    g.hub_radius_m = p.hub_radius_m;
    g.tip_loss = p.tip_loss;

    const std::size_t n = p.r_over_R.size();
    g.stations.clear();
    g.stations.reserve(opt.resample ? opt.resample_N : n);

    auto push_station = [&](double r_norm, std::size_t idx) {
        BladeStation s;
        s.r_m = r_norm * p.radius_m;
        s.chord_m = p.chord_m[idx];
        s.twist_rad = p.twist_rad[idx];
        g.stations.push_back(s);
    };

    if (!opt.resample) {
        for (std::size_t i = 0; i < n; ++i) push_station(p.r_over_R[i], i);
    } else {
        // Uniform resample in r/R from hub tip margin to 1.0
        const double start = p.r_over_R.front();
        const double end = p.r_over_R.back();
        for (std::size_t i = 0; i < opt.resample_N; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(opt.resample_N - 1);
            const double r_norm = start + t * (end - start);
            // nearest-neighbor for simplicity (can upgrade to linear later)
            std::size_t idx = static_cast<std::size_t>(std::lower_bound(p.r_over_R.begin(), p.r_over_R.end(), r_norm) - p.r_over_R.begin());
            if (idx >= n) idx = n - 1;
            push_station(r_norm, idx);
        }
    }

    g.validate();
    return g;
}

// ------------------------------
// Airfoil mapping policy
// ------------------------------
enum class AirfoilMapping : std::uint8_t {
    // Use a single representative airfoil for all stations (fast baseline).
    Single = 0,

    // Piecewise by station (airfoil_id[i] may differ). Requires a multi-polar sampler.
    Piecewise
};

// ------------------------------
// Multi-polar sampler for piecewise airfoils
// ------------------------------
class PiecewisePolar final : public IAirfoilPolar {
public:
    struct Node final {
        double r_m = 0.0; // station radius
        std::shared_ptr<const IAirfoilPolar> polar;
    };

    // Nodes must be strictly increasing in r_m. Sampling uses nearest-node selection by radius.
    explicit PiecewisePolar(std::vector<Node> nodes)
        : nodes_(std::move(nodes)) {
        if (nodes_.empty()) {
            throw BemtError(ErrorCode::InvalidInput, "PiecewisePolar: empty nodes");
        }
        for (std::size_t i = 0; i < nodes_.size(); ++i) {
            if (!nodes_[i].polar) {
                throw BemtError(ErrorCode::InvalidInput, "PiecewisePolar: null polar");
            }
            if (!is_finite(nodes_[i].r_m) || nodes_[i].r_m <= 0.0) {
                throw BemtError(ErrorCode::InvalidInput, "PiecewisePolar: invalid radius");
            }
            if (i > 0 && nodes_[i].r_m <= nodes_[i-1].r_m) {
                throw BemtError(ErrorCode::InvalidInput, "PiecewisePolar: radii not strictly increasing");
            }
        }
    }

    PolarOutput sample(const PolarQuery& q) const override {
        // No radius context; fall back to mid-span
        return sample_at_radius(nodes_.back().r_m * 0.5, q);
    }

    // Overload: sample with radius context (used by solver wrapper).
    PolarOutput sample_at_radius(double r_m, const PolarQuery& q) const {
        if (!is_finite(r_m)) r_m = nodes_.back().r_m * 0.5;
        // Nearest-node selection (deterministic)
        const Node* best = &nodes_.front();
        double bestd = std::abs(nodes_.front().r_m - r_m);
        for (const auto& n : nodes_) {
            const double d = std::abs(n.r_m - r_m);
            if (d < bestd) { bestd = d; best = &n; }
        }
        return best->polar->sample(q);
    }

private:
    std::vector<Node> nodes_;
};

// A thin wrapper to allow BEMT solver to call a polar with radius context, without changing solver signature.
// If you use Piecewise mapping, you should use this class (and pass it to BemtSolver / BemtForwardSolver).
class RadiusAwarePolar final : public IAirfoilPolar {
public:
    explicit RadiusAwarePolar(const PiecewisePolar& pp) : pp_(pp) {}

    // Standard interface (falls back to mid-radius selection if radius not set)
    PolarOutput sample(const PolarQuery& q) const override {
        return pp_.sample_at_radius(r_m_, q);
    }

    // Set context radius before station evaluation (thread-unsafe if shared; intended for per-thread instances).
    void set_radius(double r_m) noexcept { r_m_ = r_m; }

private:
    const PiecewisePolar& pp_;
    double r_m_ = std::numeric_limits<double>::quiet_NaN();
};

} // namespace lift::bemt
