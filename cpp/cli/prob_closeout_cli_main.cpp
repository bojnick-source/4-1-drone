/*
===============================================================================
Fragment 3.3.10 — Probability Closeout CLI (Top-N + MC + Emit CSVs) (C++)
File: prob_closeout_cli_main.cpp
===============================================================================

Demonstrates the probability closeout integration using stubs.
Replace:
- demo_closeout_rows()
- demo_gonogo_reports()
- baseline_inputs map
- runner_stub()
with real pipeline data + real BEMT runner.

===============================================================================
*/

#include "engine/physics/bemt_all.hpp"
#include "engine/physics/prob_closeout_integration.hpp"

#include <iostream>
#include <unordered_map>

using namespace lift::bemt;
using namespace lift::bemt::prob;

namespace {

std::vector<CloseoutRow> demo_closeout_rows() {
    CloseoutRow a;
    a.case_id = "caseA";
    a.hover_code = ErrorCode::Ok;
    a.hover_P_W = 45000.0;
    a.hover_FM = 0.65;

    CloseoutRow b = a;
    b.case_id = "caseB";
    b.hover_P_W = 52000.0;
    b.hover_FM = 0.60;

    return {a, b};
}

std::vector<GoNoGoReport> demo_gonogo_reports() {
    GoNoGoReport a;
    a.case_id = "caseA";
    a.status = GoNoGoStatus::Go;

    GoNoGoReport b;
    b.case_id = "caseB";
    b.status = GoNoGoStatus::Go;

    return {a, b};
}

McOutputs runner_stub(const McInputs& in) {
    McOutputs y;
    y.code = ErrorCode::Ok;

    const double omega = std::max(0.0, in.omega_rad_s);
    const double rs = std::max(0.5, in.radius_scale);
    const double cs = std::max(0.5, in.chord_scale);

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
        const auto closeout_rows = demo_closeout_rows();
        const auto gonogo = demo_gonogo_reports();

        std::unordered_map<std::string, McInputs> baselines;

        McInputs base;
        base.rho = 1.225;
        base.mu  = 1.81e-5;
        base.omega_rad_s = 300.0;
        base.collective_rad = 0.20;
        base.radius_scale = 1.0;
        base.chord_scale = 1.0;

        baselines["caseA"] = base;
        baselines["caseB"] = base;

        ProbCloseoutConfig cfg;

        cfg.promote.top_n = 2;
        cfg.promote.require_go = true;
        cfg.promote.sort = ProbPromoteSort::LowestHoverPower;

        cfg.mc.sampler.n = 1500;
        cfg.mc.sampler.seed = 1; // overwritten per-case by integration
        cfg.mc.required_thrust_N = 1100.0;
        cfg.mc.hover_power_cap_W = 50000.0;

        cfg.mc.dists["rho"] = normal(1.225, 0.03, 1.10, 1.35);
        cfg.mc.dists["omega"] = normal(300.0, 8.0, 260.0, 340.0);
        cfg.mc.dists["radius_scale"] = normal(1.0, 0.01, 0.95, 1.05);
        cfg.mc.dists["chord_scale"]  = normal(1.0, 0.02, 0.90, 1.10);

        cfg.seed_base = 999;

        // Prob gates
        {
            ProbGate g;
            g.key = "P_thrust_margin_ge0";
            g.metric = "thrust_margin_N";
            g.kind = ProbGateKind::ExceedGE;
            g.threshold = 0.0;
            g.p_min = 0.95;
            cfg.gates.push_back(g);
        }
        {
            ProbGate g;
            g.key = "P_power_margin_ge0";
            g.metric = "power_margin_W";
            g.kind = ProbGateKind::ExceedGE;
            g.threshold = 0.0;
            g.p_min = 0.90;
            cfg.gates.push_back(g);
        }

        const auto out = run_probability_closeout(closeout_rows, gonogo, baselines, runner_stub, cfg);

        std::cout << "===== prob_closeout.csv =====\n";
        std::cout << out.prob_closeout_csv << "\n\n";

        std::cout << "===== prob_gates.csv =====\n";
        std::cout << out.prob_gates_csv << "\n\n";

        std::cout << "n_candidates=" << out.n_candidates
                  << " n_promoted=" << out.n_promoted
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
