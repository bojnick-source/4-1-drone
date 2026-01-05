#pragma once
/*
===============================================================================
Fragment 3.1.10 — Hover BEMT Solver (C++)
File: bemt_solver.hpp
===============================================================================
*/

#include "engine/physics/bemt_types.hpp"
#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace lift::bemt {

// Hover-only BEMT solver (V_inf axial allowed; forward handled in bemt_forward.*)
// - Iterates induced velocity vi via fixed-point momentum closure.
// - Optional collective trim to target thrust.
// - Returns station-by-station outputs for auditability.

class BemtSolver final {
public:
    explicit BemtSolver(const IAirfoilPolar& polar) : polar_(polar) {}

    BemtResult solve(const BemtInputs& in) const;

private:
    BemtResult solve_at_collective_(const RotorGeometry& g,
                                   const Environment& e,
                                   const OperatingPoint& op,
                                   const SolverConfig& cfg,
                                   double collective_offset_rad,
                                   double vi_init_mps) const;

    static double prandtl_tip_loss_(std::size_t B, double r, double R, double phi_rad) noexcept;
    static double station_dr_(const RotorGeometry& g, std::size_t i) noexcept;

    static double induced_update_(double T, double rho, double A, double vax) noexcept;

private:
    const IAirfoilPolar& polar_;
};

} // namespace lift::bemt
