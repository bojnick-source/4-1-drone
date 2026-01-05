/*
===============================================================================
Fragment 3.1.11 — Forward-Flight BEMT Wrapper (Azimuthal Sweep + Swirl-Free Approx)
File: bemt_forward.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/bemt_types.hpp"
#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/bemt_require.hpp"

#include <cstddef>
#include <cstdint>

namespace lift::bemt {

// Forward flight config:
// - V_inplane is passed separately (in-plane freestream component, m/s).
// - V_axial_mps models climb/descent (positive down through rotor).
// - n_psi: azimuth samples (>= 8).
struct ForwardConfig final {
    double V_axial_mps = 0.0;
    std::size_t n_psi = 24;

    // Clamp local inflow angle for numerical stability
    double min_phi_rad = deg2rad(0.25);
    double max_phi_rad = deg2rad(89.0);

    // Optional induced-velocity iteration controls
    std::size_t max_iter_vi = 40;
    double tol_vi = 1e-3;
    double relax_vi = 0.35;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(V_axial_mps) && std::abs(V_axial_mps) < 200.0, ErrorCode::InvalidInput, "ForwardConfig.V_axial_mps invalid");
        LIFT_BEMT_REQUIRE(n_psi >= 8 && n_psi <= 720, ErrorCode::InvalidInput, "ForwardConfig.n_psi invalid");

        LIFT_BEMT_REQUIRE(is_finite(min_phi_rad) && is_finite(max_phi_rad) && min_phi_rad > 0.0 && max_phi_rad < kPi * 0.5,
                          ErrorCode::InvalidInput, "ForwardConfig phi clamp invalid");

        LIFT_BEMT_REQUIRE(max_iter_vi >= 1 && max_iter_vi <= 100000, ErrorCode::InvalidInput, "ForwardConfig.max_iter_vi invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_vi) && tol_vi > 0.0 && tol_vi < 10.0, ErrorCode::InvalidInput, "ForwardConfig.tol_vi invalid");
        LIFT_BEMT_REQUIRE(is_finite(relax_vi) && relax_vi > 0.0 && relax_vi <= 1.0, ErrorCode::InvalidInput, "ForwardConfig.relax_vi invalid");
    }
};

struct ForwardResult final {
    ErrorCode code = ErrorCode::Ok;

    double thrust_N = 0.0;
    double torque_Nm = 0.0;
    double power_W = 0.0;

    double induced_velocity_mps = 0.0;

    std::size_t vi_iters = 0;
};

class BemtForwardSolver final {
public:
    explicit BemtForwardSolver(const IAirfoilPolar& polar) : polar_(polar) {}

    ForwardResult solve(const RotorGeometry& g,
                        const Environment& e,
                        const OperatingPoint& op_hover_like,
                        const SolverConfig& cfg,
                        double V_inplane_mps,
                        const ForwardConfig& fwd) const;

private:
    static double prandtl_tip_loss_(std::size_t B, double r, double R, double phi_rad) noexcept;
    static double station_dr_(const RotorGeometry& g, std::size_t i) noexcept;

private:
    const IAirfoilPolar& polar_;
};

} // namespace lift::bemt
