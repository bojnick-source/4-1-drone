/*
===============================================================================
Fragment 3.1.07 — Airfoil Polar Interface + Linear/Tabulated Models (C++)
File: airfoil_polar.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/bemt_types.hpp"
#include "engine/physics/bemt_require.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace lift::bemt {

// Query to a polar model.
struct PolarQuery final {
    double aoa_rad = 0.0;
    double reynolds = 0.0;
    double mach = 0.0; // reserved (optional)

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(aoa_rad) && std::abs(aoa_rad) < deg2rad(180.0), ErrorCode::InvalidInput, "PolarQuery.aoa_rad invalid");
        LIFT_BEMT_REQUIRE(is_finite(reynolds) && reynolds >= 0.0, ErrorCode::InvalidInput, "PolarQuery.reynolds invalid");
        LIFT_BEMT_REQUIRE(is_finite(mach) && mach >= 0.0 && mach < 5.0, ErrorCode::InvalidInput, "PolarQuery.mach invalid");
    }
};

struct PolarOutput final {
    double cl = 0.0;
    double cd = 0.0;
};

// Abstract polar provider.
class IAirfoilPolar {
public:
    virtual ~IAirfoilPolar() = default;
    virtual PolarOutput sample(const PolarQuery& q) const = 0;
};

// -----------------------------
// LinearPolar (robust fallback)
// -----------------------------
// CL = cl0 + cla*(aoa)
// CD = cd0 + k*CL^2
// Simple soft-stall clamp: aoa beyond stall is clamped to ±stall for CL evaluation.
struct LinearPolarParams final {
    double cl0 = 0.0;
    double cla_per_rad = 2.0 * kPi;   // thin airfoil
    double cd0 = 0.01;
    double k = 0.02;
    double aoa_stall_rad = deg2rad(15.0);

    // Optional hard clamps for numerical sanity
    double cl_min = -2.0;
    double cl_max =  2.0;
    double cd_min =  0.0;
    double cd_max =  2.0;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(cl0), ErrorCode::InvalidInput, "LinearPolarParams.cl0 invalid");
        LIFT_BEMT_REQUIRE(is_finite(cla_per_rad) && std::abs(cla_per_rad) < 1000.0, ErrorCode::InvalidInput, "LinearPolarParams.cla invalid");
        LIFT_BEMT_REQUIRE(is_finite(cd0) && cd0 >= 0.0, ErrorCode::InvalidInput, "LinearPolarParams.cd0 invalid");
        LIFT_BEMT_REQUIRE(is_finite(k) && k >= 0.0, ErrorCode::InvalidInput, "LinearPolarParams.k invalid");
        LIFT_BEMT_REQUIRE(is_finite(aoa_stall_rad) && aoa_stall_rad > 0.0 && aoa_stall_rad < deg2rad(60.0), ErrorCode::InvalidInput, "LinearPolarParams.stall invalid");
        LIFT_BEMT_REQUIRE(is_finite(cl_min) && is_finite(cl_max) && cl_min < cl_max, ErrorCode::InvalidInput, "LinearPolarParams.cl bounds invalid");
        LIFT_BEMT_REQUIRE(is_finite(cd_min) && is_finite(cd_max) && cd_min <= cd_max, ErrorCode::InvalidInput, "LinearPolarParams.cd bounds invalid");
    }
};

class LinearPolar final : public IAirfoilPolar {
public:
    explicit LinearPolar(LinearPolarParams p) : p_(std::move(p)) { p_.validate(); }

    PolarOutput sample(const PolarQuery& q) const override {
        q.validate();

        const double a = clamp(q.aoa_rad, -p_.aoa_stall_rad, p_.aoa_stall_rad);
        double cl = p_.cl0 + p_.cla_per_rad * a;
        cl = clamp(cl, p_.cl_min, p_.cl_max);

        double cd = p_.cd0 + p_.k * cl * cl;
        cd = clamp(cd, p_.cd_min, p_.cd_max);

        return PolarOutput{cl, cd};
    }

private:
    LinearPolarParams p_;
};

// -----------------------------
// TabulatedPolar (AoA × Re grid)
// -----------------------------
enum class PolarOORPolicy : std::uint8_t {
    Clamp = 0,  // clamp AoA/Re into table bounds
    Throw = 1   // throw PolarOutOfRange if outside
};

struct TabulatedPolar final {
    // Strictly increasing axes.
    std::vector<double> aoa_rad;     // size Na
    std::vector<double> reynolds;    // size Nr

    // Row-major: index = ia*Nr + ir
    std::vector<double> cl;
    std::vector<double> cd;

    PolarOORPolicy policy = PolarOORPolicy::Clamp;

    void validate() const {
        LIFT_BEMT_REQUIRE(aoa_rad.size() >= 2 && reynolds.size() >= 2, ErrorCode::MissingPolarData, "TabulatedPolar axes too small");

        for (std::size_t i = 1; i < aoa_rad.size(); ++i) {
            LIFT_BEMT_REQUIRE(is_finite(aoa_rad[i]) && aoa_rad[i] > aoa_rad[i - 1], ErrorCode::MissingPolarData, "aoa_rad must be strictly increasing");
        }
        for (std::size_t i = 1; i < reynolds.size(); ++i) {
            LIFT_BEMT_REQUIRE(is_finite(reynolds[i]) && reynolds[i] > reynolds[i - 1], ErrorCode::MissingPolarData, "reynolds must be strictly increasing");
        }

        const std::size_t Na = aoa_rad.size();
        const std::size_t Nr = reynolds.size();
        const std::size_t N = Na * Nr;

        LIFT_BEMT_REQUIRE(cl.size() == N && cd.size() == N, ErrorCode::MissingPolarData, "TabulatedPolar cl/cd size mismatch");
        for (std::size_t k = 0; k < N; ++k) {
            LIFT_BEMT_REQUIRE(is_finite(cl[k]) && is_finite(cd[k]) && cd[k] >= 0.0, ErrorCode::MissingPolarData, "TabulatedPolar cl/cd contains invalid values");
        }
    }

    PolarOutput sample(const PolarQuery& q) const;
};

// Convenience wrapper implementing IAirfoilPolar.
class TabulatedPolarModel final : public IAirfoilPolar {
public:
    explicit TabulatedPolarModel(TabulatedPolar t) : t_(std::move(t)) { t_.validate(); }
    PolarOutput sample(const PolarQuery& q) const override { return t_.sample(q); }

private:
    TabulatedPolar t_;
};

} // namespace lift::bemt
