/*
===============================================================================
Fragment 3.1.21 — Closeout CLI (Dual CSV: Closeout + GO/NO-GO) (C++)
File: bemt_cli_closeout_dual_main.cpp
===============================================================================
*/

#include "engine/physics/bemt_all.hpp"

#include <iostream>
#include <vector>

using namespace lift::bemt;

static RotorGeometry demo_geometry() {
    RotorGeometry g;
    g.blade_count = 2;
    g.radius_m = 0.50;
    g.hub_radius_m = 0.06;
    g.tip_loss = TipLossModel::Prandtl;

    // Minimal 5-station blade (ascending r), placeholder geometry for compile proof.
    g.stations = {
        {0.10, 0.06, deg2rad(12.0)},
        {0.20, 0.06, deg2rad(10.0)},
        {0.30, 0.055, deg2rad(8.0)},
        {0.40, 0.050, deg2rad(6.0)},
        {0.48, 0.045, deg2rad(4.0)}
    };
    return g;
}

int main() {
    try {
        // Polar: use LinearPolar as safe fallback until tabulated data is wired.
        LinearPolarParams lp;
        lp.cl0 = 0.0;
        lp.cla_per_rad = 2.0 * kPi;
        lp.cd0 = 0.012;
        lp.k = 0.02;
        lp.aoa_stall_rad = deg2rad(15.0);
        LinearPolar polar(lp);

        // Baseline inputs
        BemtInputs in;
        in.geom = demo_geometry();
        in.env = Environment{};
        in.op = OperatingPoint{};
        in.cfg = SolverConfig{};

        // Example hover operating point
        in.op.V_inf = 0.0;
        in.op.omega_rad_s = 450.0;                 // rad/s
        in.op.collective_offset_rad = deg2rad(6.0);
        in.op.target_thrust_N.reset();             // no trim in demo

        // Closeout case list
        CloseoutCase c1;
        c1.case_id = "demo_hover_only";
        c1.hover_in = in;
        c1.run_forward = false;
        c1.run_sensitivity = true;

        CloseoutCase c2 = c1;
        c2.case_id = "demo_hover_plus_forward";
        c2.run_forward = true;
        c2.V_inplane_mps = 20.0;
        c2.forward_cfg = ForwardConfig{};
        c2.forward_cfg.V_axial_mps = 0.0;
        c2.forward_cfg.n_psi = 24;

        std::vector<CloseoutCase> cases{c1, c2};

        // Run closeout
        CloseoutRunner runner(polar);
        const auto rows = runner.run(cases, /*kT_for_sizing=*/1.15);

        // GO/NO-GO thresholds (example defaults; set per project)
        GoNoGoThresholds t;
        t.delta_mass_max_kg = 0.0;          // disabled in demo (mass ledger not wired)
        t.A_total_min_m2 = 0.0;             // disabled in demo
        t.disk_loading_max_N_m2 = 0.0;      // disabled in demo
        t.hover_power_max_W = 0.0;          // disabled in demo
        t.fm_min = 0.0;                     // disabled in demo

        GoNoGoEvaluator eval(t);

        std::vector<GoNoGoReport> reports;
        reports.reserve(rows.size());
        for (const auto& r : rows) {
            reports.push_back(eval.evaluate(r));
        }

        // Emit CSV to stdout (two sections)
        const std::string closeout = closeout_csv(rows);
        const std::string gonogo = gonogo_csv(reports);

        std::cout << "===== closeout.csv =====\n";
        std::cout << closeout << "\n";
        std::cout << "===== gonogo.csv =====\n";
        std::cout << gonogo << "\n";

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
