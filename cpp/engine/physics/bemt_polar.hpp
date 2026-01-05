#pragma once

#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/bemt.hpp"

#include <algorithm>
#include <map>
#include <tuple>
#include <vector>

namespace lift::bemt {

// A hardened tabular polar:
// - supports multiple (Re, Mach) "slices"
// - interpolates alpha within slice, and bilinear across (Re, Mach) if multiple present
// - clamps alpha to available range (or to configured bounds by caller)
class TabularPolar final : public IAirfoilPolar {
public:
    struct SliceKey final {
        double reynolds = 0.0;
        double mach = 0.0;
        bool operator<(const SliceKey& o) const noexcept {
            if (reynolds < o.reynolds) return true;
            if (reynolds > o.reynolds) return false;
            return mach < o.mach;
        }
    };

    struct Slice final {
        // alpha in radians, strictly increasing
        std::vector<double> alpha_rad;
        std::vector<double> cl;
        std::vector<double> cd;

        void validate() const;
    };

    // Add a slice at (Re, Mach)
    void add_slice(double reynolds, double mach, Slice s);

    // Finalize validates global internal state (must be called before use)
    void finalize();

    PolarOutput sample(const PolarQuery& q) const override;

    bool finalized() const noexcept { return finalized_; }

private:
    PolarOutput sample_in_slice_(const Slice& s, double alpha_rad) const;

    static double lerp_(double a, double b, double t) noexcept { return a + (b - a) * t; }

    // Find nearest bracketing keys for Re and Mach independently, then bilinear if possible.
    // If only one slice exists, returns it.
    std::vector<std::pair<SliceKey, const Slice*>> nearest_slices_(double reynolds, double mach) const;

private:
    bool finalized_ = false;
    std::map<SliceKey, Slice> slices_;
};

} // namespace lift::bemt
