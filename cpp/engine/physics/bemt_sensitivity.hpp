/*
===============================================================================
Fragment 3.1.16 — Sensitivity Analyzer (Central Differences, Normalized dT/dP) (C++)
File: bemt_sensitivity.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/bemt.hpp"
#include "engine/physics/bemt_require.hpp"
#include "engine/physics/bemt_solver.hpp"

#include <cstddef>
#include <cstdint>

namespace lift::bemt {

// Normalized sensitivity:
// n_dT = (x/T) * dT/dx, n_dP = (x/P) * dP/dx  (safe_div guarded)
struct NormalizedSens final {
  double n_dT = 0.0;
  double n_dP = 0.0;
};

struct SensitivityConfig final {
  bool central_difference = true;

  // Step sizes (relative for most, absolute for collective)
  double h_omega_rel = 0.005;          // 0.5%
  double h_rho_rel = 0.02;             // 2%
  double h_radius_rel = 0.005;         // 0.5%
  double h_chord_rel = 0.01;           // 1%
  double h_collective_abs_rad = deg2rad(0.25);

  // If true, keep target_thrust_N during perturbations (slower, mixes trim effects).
  bool allow_trim = false;

  void validate() const {
    LIFT_BEMT_REQUIRE(is_finite(h_omega_rel) && h_omega_rel > 0.0 && h_omega_rel < 0.5, ErrorCode::InvalidInput, "h_omega_rel invalid");
    LIFT_BEMT_REQUIRE(is_finite(h_rho_rel) && h_rho_rel > 0.0 && h_rho_rel < 1.0, ErrorCode::InvalidInput, "h_rho_rel invalid");
    LIFT_BEMT_REQUIRE(is_finite(h_radius_rel) && h_radius_rel > 0.0 && h_radius_rel < 0.5, ErrorCode::InvalidInput, "h_radius_rel invalid");
    LIFT_BEMT_REQUIRE(is_finite(h_chord_rel) && h_chord_rel > 0.0 && h_chord_rel < 1.0, ErrorCode::InvalidInput, "h_chord_rel invalid");
    LIFT_BEMT_REQUIRE(is_finite(h_collective_abs_rad) && h_collective_abs_rad > 0.0 && h_collective_abs_rad < deg2rad(10.0),
                      ErrorCode::InvalidInput, "h_collective_abs_rad invalid");
  }
};

struct SensitivityResult final {
  ErrorCode code = ErrorCode::Ok;
  NormalizedSens omega;
  NormalizedSens rho_sens;
  NormalizedSens radius_scale;
  NormalizedSens chord_scale;
  NormalizedSens collective;
};

class SensitivityAnalyzer final {
public:
  explicit SensitivityAnalyzer(const IAirfoilPolar& polar) : polar_(polar), solver_(polar) {}

  SensitivityResult compute(const BemtInputs& in, const SensitivityConfig& cfg) const;

private:
  static RotorGeometry scale_geom_(const RotorGeometry& g, double radius_scale, double chord_scale);
  static NormalizedSens norm_from_(double x, double T0, double P0, double dTdx, double dPdx) noexcept;
  BemtResult solve_(BemtInputs in, bool allow_trim) const;

private:
  const IAirfoilPolar& polar_;
  BemtSolver solver_;
};

} // namespace lift::bemt
