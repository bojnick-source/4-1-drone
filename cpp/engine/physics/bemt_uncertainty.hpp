/*
===============================================================================
Fragment 3.1.12 — Uncertainty Monte Carlo Driver (C++)
File: bemt_uncertainty.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/bemt_types.hpp"
#include "engine/physics/bemt_solver.hpp"
#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/stats_hooks.hpp"
#include "engine/physics/bemt_require.hpp"

#include <cstddef>
#include <cstdint>

namespace lift::bemt {

struct UncertaintyConfig final {
    std::size_t samples = 2000;
    std::uint64_t seed = 0xA5A5A5A5ULL;

    double sigma_rho = 0.02;          // relative (2%)
    double sigma_mu = 0.05;           // relative (5%)
    double sigma_omega = 0.01;        // relative (1%)
    double sigma_collective = deg2rad(0.25); // absolute rad
    double sigma_radius_scale = 0.005; // relative (0.5%)
    double sigma_chord_scale = 0.01;   // relative (1%)

    bool accept_only_ok = true;
    bool disable_trim = true;

    void validate() const {
        LIFT_BEMT_REQUIRE(samples >= 10 && samples <= 5000000, ErrorCode::InvalidInput, "UncertaintyConfig.samples invalid");
        LIFT_BEMT_REQUIRE(is_finite(sigma_rho) && sigma_rho >= 0.0 && sigma_rho < 0.5, ErrorCode::InvalidInput, "sigma_rho invalid");
        LIFT_BEMT_REQUIRE(is_finite(sigma_mu) && sigma_mu >= 0.0 && sigma_mu < 2.0, ErrorCode::InvalidInput, "sigma_mu invalid");
        LIFT_BEMT_REQUIRE(is_finite(sigma_omega) && sigma_omega >= 0.0 && sigma_omega < 0.5, ErrorCode::InvalidInput, "sigma_omega invalid");
        LIFT_BEMT_REQUIRE(is_finite(sigma_collective) && sigma_collective >= 0.0 && sigma_collective < deg2rad(10.0),
                          ErrorCode::InvalidInput, "sigma_collective invalid");
        LIFT_BEMT_REQUIRE(is_finite(sigma_radius_scale) && sigma_radius_scale >= 0.0 && sigma_radius_scale < 0.2,
                          ErrorCode::InvalidInput, "sigma_radius_scale invalid");
        LIFT_BEMT_REQUIRE(is_finite(sigma_chord_scale) && sigma_chord_scale >= 0.0 && sigma_chord_scale < 0.5,
                          ErrorCode::InvalidInput, "sigma_chord_scale invalid");
    }
};

struct UncertaintyResult final {
    ErrorCode code = ErrorCode::Ok;
    std::size_t attempted = 0;
    std::size_t accepted = 0;

    stats::UncertaintyReport report;

    explicit UncertaintyResult(std::size_t cap = 8192, std::uint64_t seed = 0x12345678ULL)
        : report(cap, seed) {}
};

class UncertaintyRunner final {
public:
    explicit UncertaintyRunner(const IAirfoilPolar& polar) : polar_(polar) {}

    UncertaintyResult run(const BemtInputs& baseline, const UncertaintyConfig& cfg) const;

private:
    static RotorGeometry scaled_geometry_(const RotorGeometry& g, double radius_scale, double chord_scale);

private:
    const IAirfoilPolar& polar_;
};

} // namespace lift::bemt
