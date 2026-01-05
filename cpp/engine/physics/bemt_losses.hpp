// ============================================================================
// Fragment 3.1.04 — BEMT Loss Models (Prandtl Tip/Root Loss + Robust Combined F) (C++)
// File: bemt_losses.hpp
// ============================================================================
//
// Purpose:
// - Provide hardened Prandtl tip and root loss factors for BEMT.
// - Avoid NaNs/infs for phi→0, r→R, r→0, and extreme exponent arguments.
// - Deterministic behavior with clamps.
//
// Definitions (standard Prandtl):
//   f_tip  = (B/2) * (R - r) / (r * sin(phi))
//   F_tip  = (2/pi) * acos( exp(-f_tip) )
//
//   f_root = (B/2) * (r - Rhub) / (r * sin(phi))
//   F_root = (2/pi) * acos( exp(-f_root) )
//
// Combined:
//  F = F_tip * F_root  (common practice; keep both toggles in solver options)
//
// Notes:
// - If r is at/above R, tip factor → 0.
// - If r is at/below Rhub, root factor → 0.
// - If sin(phi) is tiny, f becomes huge → exp(-f)≈0 → acos(0)=pi/2 → F≈1 (safe).
//
// ============================================================================

#pragma once
#include "bemt_safety.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace lift::bemt {

// Guard to avoid division blow-ups when r is tiny
inline double safe_r_denom(double r_m, double r_min = 1e-6) noexcept {
    return (r_m < r_min) ? r_min : r_m;
}

// Guard for sin(phi) (phi in radians)
inline double safe_sinphi(double phi_rad, double sin_min = 1e-8) noexcept {
    const double s = std::sin(phi_rad);
    const double a = std::fabs(s);
    if (a < sin_min) return (s < 0.0 ? -sin_min : sin_min);
    return s;
}

// Compute Prandtl F from f (robust)
inline double prandtl_F_from_f(double f) noexcept {
    if (!is_finite(f) || f < 0.0) return 1.0; // conservative: no loss
    // exp(-f) in [0,1]
    const double e = safe_exp(-f, 0.0); // cap at exp(0)=1
    // acos(e) with e clamped
    const double a = safe_acos(clamp(e, 0.0, 1.0));
    const double F = (2.0 / kPi) * a;
    // Numerical clamp
    return clamp(F, 0.0, 1.0);
}

// Tip loss factor
inline double prandtl_tip_loss(int blades,
                               double r_m,
                               double R_m,
                               double phi_rad) noexcept {
    if (blades < 2) return 1.0;
    if (!is_finite(r_m) || !is_finite(R_m) || !is_finite(phi_rad)) return 1.0;
    if (R_m <= 0.0) return 1.0;
    if (r_m >= R_m) return 0.0;

    const double r = safe_r_denom(r_m);
    const double sphi = safe_sinphi(phi_rad);
    const double num = std::max(R_m - r_m, 0.0);
    const double f = (0.5 * static_cast<double>(blades)) * safe_div(num, r * std::fabs(sphi), 1e-18);

    return prandtl_F_from_f(f);
}

// Root loss factor
inline double prandtl_root_loss(int blades,
                                double r_m,
                                double Rhub_m,
                                double phi_rad) noexcept {
    if (blades < 2) return 1.0;
    if (!is_finite(r_m) || !is_finite(Rhub_m) || !is_finite(phi_rad)) return 1.0;
    if (Rhub_m < 0.0) return 1.0;
    if (r_m <= Rhub_m) return 0.0;

    const double r = safe_r_denom(r_m);
    const double sphi = safe_sinphi(phi_rad);
    const double num = std::max(r_m - Rhub_m, 0.0);
    const double f = (0.5 * static_cast<double>(blades)) * safe_div(num, r * std::fabs(sphi), 1e-18);

    return prandtl_F_from_f(f);
}

// Combined loss factor (with toggles)
struct LossFactorOut final {
    double F_tip = 1.0;
    double F_root = 1.0;
    double F = 1.0;
};

inline LossFactorOut prandtl_losses(int blades,
                                    double r_m,
                                    double Rhub_m,
                                    double R_m,
                                    double phi_rad,
                                    bool enable_tip,
                                    bool enable_root) noexcept {
    LossFactorOut out;

    if (enable_tip)  out.F_tip  = prandtl_tip_loss(blades, r_m, R_m, phi_rad);
    if (enable_root) out.F_root = prandtl_root_loss(blades, r_m, Rhub_m, phi_rad);

    out.F = clamp(out.F_tip * out.F_root, 0.0, 1.0);
    return out;
}

} // namespace lift::bemt
