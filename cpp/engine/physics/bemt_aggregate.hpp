// ============================================================================
// Fragment 3.1.11 — BEMT Section Aggregation (Integrate dT/dQ, Build Sections, Deterministic Totals) (C++)
// File: bemt_aggregate.hpp
// ============================================================================
//
// Purpose:
// - Standardize how per-station section outputs are accumulated into totals.
// - Optional: build “sections array” for debug/closeout without changing solver logic.
// - Deterministic summation (Kahan-style optional) to reduce sensitivity to station count.
//
// This is intended to be called by your existing bemt_solver / bemt_forward after each station solve.
// It does not compute aerodynamics; it only sums what you feed it.
//
// ============================================================================

#pragma once
#include "bemt_safety.hpp"

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace lift::bemt {

// Minimal section record for aggregation. Map from your existing per-station output.
struct AggSection final {
    double r_m = 0.0;
    double dr_m = 0.0;

    double phi_rad = 0.0;
    double alpha_rad = 0.0;
    double Re = 0.0;

    double cl = 0.0;
    double cd = 0.0;

    double dT_N = 0.0;   // total rotor contribution for this annulus
    double dQ_Nm = 0.0;  // total rotor contribution for this annulus
};

struct AggTotals final {
    double T_N = 0.0;
    double Q_Nm = 0.0;

    // Kahan compensation
    double cT = 0.0;
    double cQ = 0.0;
};

inline void kahan_add(double x, double& sum, double& c) noexcept {
    // Kahan compensated summation
    double y = x - c;
    double t = sum + y;
    c = (t - sum) - y;
    sum = t;
}

struct AggConfig final {
    bool use_kahan = true;
    bool keep_sections = false;

    // If true, invalid dT/dQ values are treated as 0 and flagged by caller.
    bool sanitize_nonfinite = true;
};

struct AggState final {
    AggConfig cfg;
    AggTotals tot;
    std::vector<AggSection> sections;

    explicit AggState(const AggConfig& c = {}) : cfg(c) {
        if (!cfg.keep_sections) sections.clear();
    }
};

inline void agg_reset(AggState& s) {
    s.tot = AggTotals{};
    s.sections.clear();
}

inline void agg_push(AggState& s, const AggSection& sec) {
    double dT = sec.dT_N;
    double dQ = sec.dQ_Nm;

    if (s.cfg.sanitize_nonfinite) {
        if (!is_finite(dT)) dT = 0.0;
        if (!is_finite(dQ)) dQ = 0.0;
    }

    if (s.cfg.use_kahan) {
        kahan_add(dT, s.tot.T_N, s.tot.cT);
        kahan_add(dQ, s.tot.Q_Nm, s.tot.cQ);
    } else {
        s.tot.T_N += dT;
        s.tot.Q_Nm += dQ;
    }

    if (s.cfg.keep_sections) {
        s.sections.push_back(sec);
    }
}

inline double agg_power_W(double Q_Nm, double omega_rad_s) noexcept {
    if (!is_finite(Q_Nm) || !is_finite(omega_rad_s) || omega_rad_s < 0.0) return 0.0;
    const double P = Q_Nm * omega_rad_s;
    return is_finite(P) ? P : 0.0;
}

} // namespace lift::bemt
