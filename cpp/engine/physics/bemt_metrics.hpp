/*
===============================================================================
Fragment 3.1.17 — Disk / Induced Power Metrics (A_total, DL, P_hover_ideal, FM)
File: bemt_metrics.hpp
===============================================================================
*/

#pragma once
#include "engine/physics/bemt_require.hpp"

#include <cmath>
#include <cstddef>

namespace lift::bemt {

// Actuator disk area (single rotor)
inline double disk_area(double radius_m) noexcept {
  if (!is_finite(radius_m) || radius_m <= 0.0) return 0.0;
  return kPi * radius_m * radius_m;
}

// Total area (sum of independent disks; coax in same duct does NOT double area)
// Caller supplies count_independent_disks.
inline double total_disk_area(double radius_m, std::size_t count_independent_disks) noexcept {
  const double A1 = disk_area(radius_m);
  if (A1 <= 0.0) return 0.0;
  if (count_independent_disks == 0) return 0.0;
  return A1 * static_cast<double>(count_independent_disks);
}

// Disk loading DL = T / A
inline double disk_loading(double thrust_N, double A_m2) noexcept {
  if (!is_finite(thrust_N) || thrust_N < 0.0) return 0.0;
  if (!is_finite(A_m2) || A_m2 <= 0.0) return 0.0;
  return safe_div(thrust_N, A_m2, 0.0);
}

// Ideal induced power for hover (actuator disk):
// P_ideal = T^(3/2) / sqrt(2*rho*A)
inline double induced_power_ideal(double thrust_N, double rho, double A_m2) noexcept {
  if (!is_finite(thrust_N) || thrust_N <= 0.0) return 0.0;
  if (!is_finite(rho) || rho <= 0.0) return 0.0;
  if (!is_finite(A_m2) || A_m2 <= 0.0) return 0.0;

  const double denom = std::sqrt(std::max(1e-18, 2.0 * rho * A_m2));
  const double num = thrust_N * std::sqrt(std::max(0.0, thrust_N));  // T^(3/2)
  const double P = safe_div(num, denom, 0.0);
  return (is_finite(P) && P >= 0.0) ? P : 0.0;
}

// Figure of Merit (FM) = P_ideal / P_actual (hover-ish)
inline double figure_of_merit(double P_ideal_W, double P_actual_W) noexcept {
  if (!is_finite(P_ideal_W) || P_ideal_W <= 0.0) return 0.0;
  if (!is_finite(P_actual_W) || P_actual_W <= 0.0) return 0.0;
  const double fm = safe_div(P_ideal_W, P_actual_W, 0.0);
  return clamp(fm, 0.0, 1.0);
}

inline double figure_of_merit_from_rho(double thrust_N, double P_actual_W, double rho, double A_m2) noexcept {
  const double Pid = induced_power_ideal(thrust_N, rho, A_m2);
  return figure_of_merit(Pid, P_actual_W);
}

// Sensitivity helper: sized hover power for thrust margin kT (>=1).
// Uses ideal induced scaling as a first-order proxy.
// P_sized(kT) ≈ (kT)^(3/2) * P_hover
inline double sized_hover_power(double P_hover_W, double kT) noexcept {
  if (!is_finite(P_hover_W) || P_hover_W < 0.0) return 0.0;
  if (!is_finite(kT) || kT <= 0.0) return P_hover_W;
  const double scale = std::pow(kT, 1.5);
  const double P = P_hover_W * scale;
  return (is_finite(P) && P >= 0.0) ? P : 0.0;
}

} // namespace lift::bemt
