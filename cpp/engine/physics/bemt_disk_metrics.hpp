#pragma once

#include "engine/physics/bemt_solver.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <limits>
#include <vector>
#include <cmath>

namespace lift::bemt {

// --------------------------------------------
// Disk / hover power metrics (closeout-ready)
// --------------------------------------------
struct DiskMetrics final {
    // Geometry
    double radius_m = 0.0;
    double area_m2 = 0.0;             // A = pi R^2
    double disk_loading_N_m2 = 0.0;   // DL = T / A

    // Hover power
    double thrust_N = 0.0;
    double power_W = 0.0;            // actual from BEMT
    double induced_ideal_W = 0.0;    // momentum theory ideal induced power
    double figure_of_merit = 0.0;    // FM = P_ideal / P_actual (clamped)

    // Optional sizing concepts
    // kT sizing = size system for k * T, power predicted using ideal scaling and BEMT slope approximation.
    double k_thrust = 1.0;           // sizing factor on thrust
    double sized_thrust_N = 0.0;     // k*T
    double sized_power_W_simple = 0.0; // conservative: P_actual * k^(3/2) (hover induced scaling)
};

// Compute disk metrics from a hover result.
// Assumes hover/axial usage; does not require V_inf=0 but intended for hover closeout.
inline DiskMetrics compute_disk_metrics(const RotorGeometry& g, const BemtResult& r, double k_thrust = 1.0) {
    g.validate();

    DiskMetrics m{};
    m.radius_m = g.radius_m;
    m.area_m2 = kPi * g.radius_m * g.radius_m;

    const double T = (is_finite(r.thrust_N) && r.thrust_N >= 0.0) ? r.thrust_N : 0.0;
    const double P = (is_finite(r.power_W) && r.power_W >= 0.0) ? r.power_W : 0.0;

    m.thrust_N = T;
    m.power_W = P;

    if (m.area_m2 > 0.0 && is_finite(m.area_m2)) {
        m.disk_loading_N_m2 = T / m.area_m2;
    } else {
        m.disk_loading_N_m2 = 0.0;
    }

    // Ideal induced power (hover)
    // P_ideal = T^(3/2) / sqrt(2*rho*A) -> rho not stored in result; ideal power should be computed outside with env.
    // Here we reuse r.figure_of_merit if solver filled it, else compute with a placeholder 0.
    m.figure_of_merit = (is_finite(r.figure_of_merit) ? clamp(r.figure_of_merit, 0.0, 1.0) : 0.0);

    // Sizing concepts (simple conservative scaling):
    // For hover induced-dominated power: P ~ T^(3/2). So P_sized ≈ P * k^(3/2).
    m.k_thrust = (is_finite(k_thrust) ? clamp(k_thrust, 0.1, 10.0) : 1.0);
    m.sized_thrust_N = m.k_thrust * T;
    m.sized_power_W_simple = (P > 0.0) ? (P * std::pow(m.k_thrust, 1.5)) : 0.0;

    // induced_ideal_W is not computed without rho; set 0 here. Use helper below.
    m.induced_ideal_W = 0.0;

    return m;
}

// Ideal induced power (momentum hover), explicit rho:
// P_ideal = T^(3/2) / sqrt(2*rho*A)
inline double induced_power_ideal_hover_W(double thrust_N, double rho, double area_m2) {
    if (!is_finite(thrust_N) || thrust_N <= 0.0) return 0.0;
    if (!is_finite(rho) || rho <= 0.0) return 0.0;
    if (!is_finite(area_m2) || area_m2 <= 0.0) return 0.0;

    const double denom = std::sqrt(2.0 * rho * area_m2);
    if (!is_finite(denom) || denom <= 0.0) return 0.0;

    const double p = std::pow(thrust_N, 1.5) / denom;
    return (is_finite(p) && p >= 0.0) ? p : 0.0;
}

// Convenience: compute FM from explicit rho
inline double figure_of_merit_from_rho(double thrust_N, double power_W, double rho, double area_m2) {
    const double pideal = induced_power_ideal_hover_W(thrust_N, rho, area_m2);
    if (pideal <= 0.0 || power_W <= 0.0) return 0.0;
    return clamp(pideal / power_W, 0.0, 1.0);
}

} // namespace lift::bemt
