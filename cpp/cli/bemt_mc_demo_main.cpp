/*
===============================================================================
Fragment 3.3.07 — Monte Carlo Probability CLI Demo (Runner Stub + CSV Emit) (C++)
File: bemt_mc_demo_main.cpp
===============================================================================

Demonstrates:
- building uncertainty distributions
- running Monte Carlo wrapper
- emitting prob_summary.csv and prob_gates.csv (printed to stdout)

Replace runner_stub() with real BEMT integration.
===============================================================================
*/

#include "engine/physics/bemt_all.hpp"
#include "engine/physics/bemt_mc.hpp"

#include <iostream>

using namespace lift::bemt;
using namespace lift::bemt::prob;

namespace {

static McOutputs runner_stub(const McInputs& in) {
    // Placeholder surrogate:
    // - Hover power increases with density and omega.
    // - Thrust increases with density, omega^2, and radius_scale^2.
    // - FM weakly degrades with higher omega.
    //
    // Replace with: BemtSolver::solve_hover() etc.

    McOutputs y;
    y.code = ErrorCode::Ok;

    const double omega = std::max(0.0, in.omega_rad_s);
    const double rs = std::max(0.5, in.radius_scale);
    const double cs = std::max(0.5, in.chord_scale);

    // crude scaling placeholders
    y.hover_T_N = 1200.0 * (in.rho / 1.225) * (omega * omega / (300.0 * 300.0)) * (rs * rs);
    y.hover_P_W = 40000.0 * (in.rho / 1.225) * (omega / 300.0) * (rs) * (0.85 + 0.15 * cs);
    y.hover_FM  = clamp(0.75 - 0.10 * ((omega - 300.0) / 300.0), 0.45, 0.80);

    if (!is_finite(y.hover_T_N) || !is_finite(y.hover_P_W) || !is_finite(y.hover_FM)) {
        y.code = ErrorCode::NumericalFailure;
    }
    return y;
}

} // namespace

int main() {
    try {
        McInputs baseline;
        baseline.rho = 1.225;
        baseline.mu  = 1.81e-5;
        baseline.omega_rad_s = 300.0;
        baseline.collective_rad = 0.20;
        baseline.radius_scale = 1.0;
        baseline.chord_scale = 1.0;

        McConfig cfg;
        cfg.sampler.n = 2000;
        cfg.sampler.seed = 42;

        // Example uncertainties (tight; tune later)
        cfg.dists["rho"] = normal(1.225, 0.03, 1.10, 1.35);
        cfg.dists["omega"] = normal(300.0, 8.0, 260.0, 340.0);
        cfg.dists["radius_scale"] = normal(1.0, 0.01, 0.95, 1.05);
        cfg.dists["chord_scale"]  = normal(1.0, 0.02, 0.90, 1.10);

        cfg.required_thrust_N = 1100.0;
        cfg.hover_power_cap_W = 50000.0;

        // Probabilistic gates (example policy)
        std::vector<ProbGate> gates;
        {
            ProbGate g;
            g.key = "P_thrust_margin_ge0";
            g.metric = "thrust_margin_N";
            g.kind = ProbGateKind::ExceedGE;
            g.threshold = 0.0;
            g.p_min = 0.95;
            gates.push_back(g);
        }
        {
            ProbGate g;
            g.key = "P_power_margin_ge0";
            g.metric = "power_margin_W";
            g.kind = ProbGateKind::ExceedGE;
            g.threshold = 0.0;
            g.p_min = 0.90;
            gates.push_back(g);
        }

        const auto res = run_bemt_monte_carlo("demo_case", baseline, runner_stub, cfg, gates);

        std::cout << "===== prob_summary.csv =====\n";
        std::cout << res.prob_summary_csv << "\n";

        if (!res.prob_gate_csv.empty()) {
            std::cout << "===== prob_gates.csv =====\n";
            std::cout << res.prob_gate_csv << "\n";
        }

        std::cout << "n_requested=" << res.n_requested
                  << " n_ok=" << res.n_ok
                  << " n_failed=" << res.n_failed
                  << "\n";

        return 0;
    } catch (const BemtError& e) {
        std::cerr << "BEMT ERROR code=" << static_cast<unsigned>(e.code())
                  << " msg=" << e.what()
                  << " at " << e.where().file << ":" << e.where().line
                  << " (" << e.where().func << ")\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "STD EXCEPTION: " << e.what() << "\n";
        return 3;
    } catch (...) {
        std::cerr << "UNKNOWN EXCEPTION\n";
        return 4;
    }
}
