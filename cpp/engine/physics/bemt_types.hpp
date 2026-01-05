/*
===============================================================================
Fragment 3.1.06 — Core Types (Geometry / Env / Operating / Config / Results) (C++)
File: bemt_types.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace lift::bemt {

// -----------------------------
// Rotor geometry
// -----------------------------
struct BladeStation final {
    // radius from hub center [m]
    double r_m = 0.0;
    // chord [m]
    double chord_m = 0.0;
    // geometric twist [rad] (positive increases local pitch)
    double twist_rad = 0.0;
    // optional airfoil id (if empty, use default/global polar)
    std::string airfoil_id;
};

enum class TipLossModel : std::uint8_t {
    None = 0,
    Prandtl = 1
};

struct RotorGeometry final {
    std::size_t blade_count = 2;

    // rotor radius [m]
    double radius_m = 0.0;
    // hub cutout radius [m] (must be < radius_m)
    double hub_radius_m = 0.0;

    TipLossModel tip_loss = TipLossModel::Prandtl;

    // Radial stations in ascending r
    std::vector<BladeStation> stations;

    void validate() const {
        LIFT_BEMT_REQUIRE(blade_count >= 2 && blade_count <= 16, ErrorCode::InvalidGeometry, "blade_count out of range");
        LIFT_BEMT_REQUIRE(is_finite(radius_m) && radius_m > 0.0, ErrorCode::InvalidGeometry, "radius_m invalid");
        LIFT_BEMT_REQUIRE(is_finite(hub_radius_m) && hub_radius_m >= 0.0, ErrorCode::InvalidGeometry, "hub_radius_m invalid");
        LIFT_BEMT_REQUIRE(hub_radius_m < radius_m, ErrorCode::InvalidGeometry, "hub_radius_m must be < radius_m");
        LIFT_BEMT_REQUIRE(stations.size() >= 3 && stations.size() <= 512, ErrorCode::InvalidGeometry, "stations size out of range");

        double prev = -1.0;
        for (const auto& s : stations) {
            LIFT_BEMT_REQUIRE(is_finite(s.r_m) && is_finite(s.chord_m) && is_finite(s.twist_rad),
                              ErrorCode::InvalidGeometry, "station contains non-finite value");
            LIFT_BEMT_REQUIRE(s.r_m > hub_radius_m && s.r_m < radius_m,
                              ErrorCode::InvalidGeometry, "station radius out of [hub, tip)");
            LIFT_BEMT_REQUIRE(s.chord_m > 0.0 && s.chord_m < 10.0, ErrorCode::InvalidGeometry, "station chord invalid");
            LIFT_BEMT_REQUIRE(s.r_m > prev, ErrorCode::InvalidGeometry, "stations must be strictly increasing in r");
            prev = s.r_m;
        }
    }
};

// -----------------------------
// Environment
// -----------------------------
struct Environment final {
    // air density [kg/m^3]
    double rho = 1.225;
    // dynamic viscosity [Pa*s]
    double mu = 1.81e-5;
    // speed of sound [m/s] (for Mach checks; optional)
    double a_m_s = 340.0;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(rho) && rho > 0.0 && rho < 5.0, ErrorCode::InvalidEnvironment, "rho invalid");
        LIFT_BEMT_REQUIRE(is_finite(mu) && mu > 0.0 && mu < 1e-2, ErrorCode::InvalidEnvironment, "mu invalid");
        LIFT_BEMT_REQUIRE(is_finite(a_m_s) && a_m_s > 0.0 && a_m_s < 2000.0, ErrorCode::InvalidEnvironment, "a_m_s invalid");
    }
};

// -----------------------------
// Operating point
// -----------------------------
struct OperatingPoint final {
    // axial freestream [m/s], positive downward through rotor
    double V_inf = 0.0;

    // rotor speed [rad/s]
    double omega_rad_s = 0.0;

    // collective pitch offset applied to all stations [rad]
    double collective_offset_rad = 0.0;

    // Optional thrust target for collective trim [N]
    std::optional<double> target_thrust_N;

    // Forward-flight inflow angle (rad) for resolving freestream components
    double inflow_angle_rad = 0.0;

    enum class FlightMode : std::uint8_t { Hover = 0, Forward = 1 };
    FlightMode mode = FlightMode::Hover;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(V_inf) && std::abs(V_inf) < 200.0, ErrorCode::InvalidOperatingPoint, "V_inf invalid");
        LIFT_BEMT_REQUIRE(is_finite(omega_rad_s) && omega_rad_s > 0.0 && omega_rad_s < 20000.0, ErrorCode::InvalidOperatingPoint, "omega invalid");
        LIFT_BEMT_REQUIRE(is_finite(collective_offset_rad) && std::abs(collective_offset_rad) < deg2rad(45.0),
                          ErrorCode::InvalidOperatingPoint, "collective_offset_rad invalid");
        LIFT_BEMT_REQUIRE(is_finite(inflow_angle_rad) && std::abs(inflow_angle_rad) < deg2rad(120.0),
                          ErrorCode::InvalidOperatingPoint, "inflow_angle_rad invalid");

        if (target_thrust_N.has_value()) {
            LIFT_BEMT_REQUIRE(is_finite(*target_thrust_N) && *target_thrust_N > 0.0,
                              ErrorCode::InvalidOperatingPoint, "target_thrust_N invalid");
        }
    }
};

// -----------------------------
// Solver configuration
// -----------------------------
struct SolverConfig final {
    // Inflow iteration
    std::size_t max_iter_inflow = 60;
    double tol_inflow = 1e-4;
    double inflow_relax = 0.35;

    // Trim iteration
    std::size_t max_iter_trim = 60;
    double tol_trim_N = 0.5;

    // Collective bounds used for trim
    double collective_min_rad = deg2rad(-5.0);
    double collective_max_rad = deg2rad(25.0);

    // Numerical clamps
    double min_phi_rad = deg2rad(0.25);
    double max_phi_rad = deg2rad(89.0);

    double min_aoa_rad = deg2rad(-25.0);
    double max_aoa_rad = deg2rad(25.0);

    // Ensure nonzero integration step
    double min_dr_m = 1e-6;

    void validate() const {
        LIFT_BEMT_REQUIRE(max_iter_inflow >= 1 && max_iter_inflow <= 100000, ErrorCode::InvalidConfig, "max_iter_inflow invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_inflow) && tol_inflow > 0.0 && tol_inflow < 10.0, ErrorCode::InvalidConfig, "tol_inflow invalid");
        LIFT_BEMT_REQUIRE(is_finite(inflow_relax) && inflow_relax > 0.0 && inflow_relax <= 1.0, ErrorCode::InvalidConfig, "inflow_relax invalid");

        LIFT_BEMT_REQUIRE(max_iter_trim >= 1 && max_iter_trim <= 100000, ErrorCode::InvalidConfig, "max_iter_trim invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol_trim_N) && tol_trim_N > 0.0 && tol_trim_N < 1e6, ErrorCode::InvalidConfig, "tol_trim_N invalid");

        LIFT_BEMT_REQUIRE(is_finite(collective_min_rad) && is_finite(collective_max_rad) && collective_min_rad < collective_max_rad,
                          ErrorCode::InvalidConfig, "collective bounds invalid");

        LIFT_BEMT_REQUIRE(is_finite(min_phi_rad) && is_finite(max_phi_rad) && min_phi_rad > 0.0 && max_phi_rad < kPi * 0.5,
                          ErrorCode::InvalidConfig, "phi clamp invalid");

        LIFT_BEMT_REQUIRE(is_finite(min_aoa_rad) && is_finite(max_aoa_rad) && min_aoa_rad < max_aoa_rad,
                          ErrorCode::InvalidConfig, "aoa clamp invalid");

        LIFT_BEMT_REQUIRE(is_finite(min_dr_m) && min_dr_m > 0.0, ErrorCode::InvalidConfig, "min_dr_m invalid");
    }
};

// -----------------------------
// Inputs / outputs
// -----------------------------
struct BemtInputs final {
    RotorGeometry geom;
    Environment env;
    OperatingPoint op;
    SolverConfig cfg;
};

struct StationResult final {
    double r_m = 0.0;
    double dr_m = 0.0;

    double aoa_rad = 0.0;
    double phi_rad = 0.0;

    double cl = 0.0;
    double cd = 0.0;

    double dT_N = 0.0;
    double dQ_Nm = 0.0;

    double V_axial_m_s = 0.0;
    double V_tan_m_s = 0.0;
    double V_rel_m_s = 0.0;

    double reynolds = 0.0;
    double mach = 0.0;

    double tip_loss_F = 1.0;
};

struct BemtResult final {
    ErrorCode code = ErrorCode::Ok;

    double thrust_N = 0.0;
    double torque_Nm = 0.0;
    double power_W = 0.0;

    double induced_velocity_m_s = 0.0;
    double figure_of_merit = 0.0;

    // Records for auditability
    double collective_offset_rad = 0.0;
    std::size_t inflow_iters = 0;
    std::size_t trim_iters = 0;

    std::vector<StationResult> stations;
};

} // namespace lift::bemt
