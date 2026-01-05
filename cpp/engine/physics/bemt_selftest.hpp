// ============================================================================
// Fragment 3.1.15 — BEMT Self-Test (Deterministic Microtests for Loss/Kinematics/Integrity/Grid) (C++)
// File: bemt_selftest.hpp
// ============================================================================
//
// Purpose:
// - Provide a zero-dependency, deterministic “microtest” suite for core BEMT helpers.
// - Verifies numerical hardening invariants WITHOUT needing full solver wiring.
// - Intended usage:
//     auto rep = lift::bemt::selftest::run_all();
//     if (!rep.ok()) { ...log rep...; }
//
// What it tests:
// - Station grid validation + dr computation
// - Prandtl tip/root loss invariants (bounds, monotonic behavior sanity)
// - Kinematics (finite outputs, basic Re behavior)
// - Integrity (FM, torque-power consistency math)
//
// Notes:
// - This is not a physics validation suite. It is a “no-NaN/no-regression” guard.
// - Keep it cheap: safe to run in CI on every build.
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"
#include "bemt_station_grid.hpp"
#include "bemt_losses.hpp"
#include "bemt_kinematics.hpp"
#include "bemt_integrity.hpp"
#include "bemt_safety.hpp"

#include <string>
#include <vector>
#include <cmath>

namespace lift::bemt::selftest {

struct Report final {
    std::vector<std::string> failures;

    bool ok() const noexcept { return failures.empty(); }

    void fail(const std::string& s) { failures.push_back(s); }
};

// Small helper: approximate equality
inline bool near(double a, double b, double rel = 1e-9, double abs = 1e-12) noexcept {
    const double da = std::fabs(a - b);
    if (da <= abs) return true;
    const double sc = std::max({std::fabs(a), std::fabs(b), abs});
    return da / sc <= rel;
}

inline void test_station_grid(Report& r) {
    try {
        std::vector<double> rs{0.10, 0.20, 0.35, 0.60};
        StationGridConfig cfg;
        cfg.require_strictly_increasing = true;
        const auto dr = compute_station_dr(rs, cfg);

        if (dr.size() != rs.size()) r.fail("station_grid: dr size mismatch");
        if (!(dr[0] > 0.0 && dr[1] > 0.0 && dr[2] > 0.0 && dr[3] > 0.0)) r.fail("station_grid: dr not positive");

        // Endpoint policy checks
        if (!near(dr[0], rs[1] - rs[0], 1e-12, 1e-12)) r.fail("station_grid: dr[0] endpoint policy mismatch");
        if (!near(dr.back(), rs.back() - rs[rs.size() - 2], 1e-12, 1e-12)) r.fail("station_grid: dr[last] endpoint policy mismatch");
    } catch (const std::exception& e) {
        r.fail(std::string("station_grid: threw: ") + e.what());
    }
}

inline void test_prandtl_losses(Report& r) {
    // Basic bounds and behavior
    const int B = 4;
    const double Rhub = 0.10;
    const double R = 1.00;

    // Typical phi
    const double phi = 0.35;

    const auto Lmid = prandtl_losses(B, 0.70, Rhub, R, phi, true, true);
    if (!(Lmid.F >= 0.0 && Lmid.F <= 1.0)) r.fail("prandtl: F out of [0,1] mid-span");

    // Near tip: F_tip should be smaller than mid-span (generally)
    const auto Ltip = prandtl_losses(B, 0.99, Rhub, R, phi, true, true);
    if (!(Ltip.F <= Lmid.F + 1e-12)) r.fail("prandtl: expected lower F near tip");

    // At tip: tip loss -> 0
    const auto Ltip0 = prandtl_losses(B, R, Rhub, R, phi, true, true);
    if (!near(Ltip0.F_tip, 0.0, 1e-12, 1e-12)) r.fail("prandtl: F_tip at r=R should be 0");

    // At hub: root loss -> 0
    const auto Lhub0 = prandtl_losses(B, Rhub, Rhub, R, phi, true, true);
    if (!near(Lhub0.F_root, 0.0, 1e-12, 1e-12)) r.fail("prandtl: F_root at r=Rhub should be 0");

    // Phi near 0: should remain finite and in [0,1]
    const auto Lphi0 = prandtl_losses(B, 0.70, Rhub, R, 1e-12, true, true);
    if (!(is_finite(Lphi0.F) && Lphi0.F >= 0.0 && Lphi0.F <= 1.0)) r.fail("prandtl: phi->0 produced invalid F");
}

inline void test_kinematics(Report& r) {
    try {
        BemtKinematicsIn in;
        in.rho_kg_m3 = 1.225;
        in.mu_Pa_s = 1.81e-5;
        in.omega_rad_s = 300.0;
        in.r_m = 0.5;
        in.chord_m = 0.08;
        in.twist_rad = 0.05;
        in.collective_rad = 0.10;
        in.v_axial_m_s = 3.0;
        in.v_inplane_m_s = 0.0;

        const auto o = bemt_kinematics(in);

        if (!is_finite(o.vrel_m_s) || o.vrel_m_s <= 0.0) r.fail("kinematics: vrel invalid");
        if (!is_finite(o.phi_rad)) r.fail("kinematics: phi not finite");
        if (!is_finite(o.alpha_rad)) r.fail("kinematics: alpha not finite");
        if (!is_finite(o.Re) || o.Re <= 0.0) r.fail("kinematics: Re invalid");

        // Ensure alpha = theta - phi (wrapped) holds approximately
        const double theta = in.twist_rad + in.collective_rad;
        const double a = wrap_pi(theta - o.phi_rad);
        if (!near(a, o.alpha_rad, 1e-12, 1e-12)) r.fail("kinematics: alpha mismatch vs wrap_pi(theta-phi)");
    } catch (const std::exception& e) {
        r.fail(std::string("kinematics: threw: ") + e.what());
    }
}

inline void test_integrity(Report& r) {
    // Construct a consistent set:
    // Choose Q and omega to match P, and T to produce FM in reasonable range.
    const double rho = 1.225;
    const double R = 1.0;
    const double A = disk_area_from_radius(R);

    const double omega = 400.0;
    const double Q = 50.0;           // Nm
    const double P = Q * omega;      // W

    const double T = 800.0;          // N (arbitrary)
    const auto out = bemt_integrity(T, Q, P, rho, A, omega);

    if (!out.ok()) r.fail("integrity: expected ok for consistent Q*omega");

    if (!is_finite(out.P_ideal_hover_W) || out.P_ideal_hover_W <= 0.0) r.fail("integrity: P_ideal invalid");
    if (!is_finite(out.FM)) r.fail("integrity: FM not finite");
    if (!(out.FM >= 0.0)) r.fail("integrity: FM negative");
    if (!is_finite(out.disk_loading_N_m2) || out.disk_loading_N_m2 <= 0.0) r.fail("integrity: disk loading invalid");

    // Power mismatch case should fail
    const auto bad = bemt_integrity(T, Q, P * 1.5, rho, A, omega);
    if (bad.ok()) r.fail("integrity: expected failure on torque-power mismatch");
}

inline Report run_all() {
    Report rep;
    test_station_grid(rep);
    test_prandtl_losses(rep);
    test_kinematics(rep);
    test_integrity(rep);
    return rep;
}

} // namespace lift::bemt::selftest
