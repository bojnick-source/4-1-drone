/*
===============================================================================
Fragment 3.1.12 — Uncertainty Monte Carlo Driver (C++)
File: bemt_uncertainty.cpp
===============================================================================
*/

#include "engine/physics/bemt_uncertainty.hpp"

#include <random>

namespace lift::bemt {

RotorGeometry UncertaintyRunner::scaled_geometry_(const RotorGeometry& g, double radius_scale, double chord_scale) {
    RotorGeometry out = g;

    radius_scale = positive_or(radius_scale, 1.0);
    chord_scale = positive_or(chord_scale, 1.0);

    out.radius_m *= radius_scale;
    out.hub_radius_m *= radius_scale;
    for (auto& s : out.stations) {
        s.r_m *= radius_scale;
        s.chord_m *= chord_scale;
    }
    return out;
}

UncertaintyResult UncertaintyRunner::run(const BemtInputs& baseline, const UncertaintyConfig& cfg) const {
    baseline.geom.validate();
    baseline.env.validate();
    baseline.op.validate();
    baseline.cfg.validate();
    cfg.validate();

    std::mt19937_64 rng(cfg.seed);
    std::normal_distribution<double> gauss(0.0, 1.0);

    BemtInputs pert = baseline;
    if (cfg.disable_trim) {
        pert.op.target_thrust_N.reset();
    }

    UncertaintyResult out;
    out.report.reset();

    for (std::size_t i = 0; i < cfg.samples; ++i) {
        ++out.attempted;

        // Draw perturbations
        const double drho = gauss(rng) * cfg.sigma_rho;
        const double dmu = gauss(rng) * cfg.sigma_mu;
        const double domega = gauss(rng) * cfg.sigma_omega;
        const double dcollective = gauss(rng) * cfg.sigma_collective;
        const double dradius = gauss(rng) * cfg.sigma_radius_scale;
        const double dchord = gauss(rng) * cfg.sigma_chord_scale;

        pert.env = baseline.env;
        pert.env.rho = baseline.env.rho * (1.0 + drho);
        pert.env.mu = baseline.env.mu * (1.0 + dmu);

        pert.op = baseline.op;
        if (cfg.disable_trim) {
            pert.op.target_thrust_N.reset();
        }
        pert.op.omega_rad_s = baseline.op.omega_rad_s * (1.0 + domega);
        pert.op.collective_offset_rad = baseline.op.collective_offset_rad + dcollective;

        const double rscale = 1.0 + dradius;
        const double cscale = 1.0 + dchord;
        pert.geom = scaled_geometry_(baseline.geom, rscale, cscale);

        // Validate perturbed inputs; skip if invalid.
        try {
            pert.geom.validate();
            pert.env.validate();
            pert.op.validate();
        } catch (...) {
            continue;
        }

        BemtSolver solver(polar_);
        const BemtResult r = solver.solve(pert);

        if (r.code != ErrorCode::Ok) {
            if (cfg.accept_only_ok) continue;
        }

        ++out.accepted;
        out.report.push_sample(r);
    }

    if (out.accepted == 0) {
        out.code = ErrorCode::NonConverged;
    }

    return out;
}

} // namespace lift::bemt
