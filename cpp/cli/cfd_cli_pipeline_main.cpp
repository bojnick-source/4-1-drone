/*
===============================================================================
Fragment 3.2.06 — CFD Pipeline CLI (Emit manifest + optional corrected outputs)
File: cfd_cli_pipeline_main.cpp
===============================================================================
*/

#include "engine/physics/bemt_all.hpp"

#include <iostream>
#include <unordered_map>
#include <vector>

using namespace lift::bemt;

namespace {

static std::vector<CloseoutRow> demo_rows() {
    // Minimal stable demo row list
    CloseoutRow r1;
    r1.case_id = "caseA";
    r1.A_m2 = 0.78539816339;
    r1.hover_code = ErrorCode::Ok;
    r1.hover_T_N = 1500.0;
    r1.hover_P_W = 45000.0;
    r1.hover_FM = 0.65;

    CloseoutRow r2 = r1;
    r2.case_id = "caseB";
    r2.hover_T_N = 1500.0;
    r2.hover_P_W = 52000.0;
    r2.hover_FM = 0.60;

    return {r1, r2};
}

static std::vector<GoNoGoReport> demo_gonogo(const std::vector<CloseoutRow>& rows) {
    std::vector<GoNoGoReport> out;
    out.reserve(rows.size());
    for (const auto& r : rows) {
        GoNoGoReport gr;
        gr.case_id = r.case_id;
        gr.status = GoNoGoStatus::Go;
        out.push_back(std::move(gr));
    }
    return out;
}

} // namespace

int main() {
    try {
        const auto rows = demo_rows();
        const auto gonogo = demo_gonogo(rows);

        // Reference maps (case_id -> BEMT values)
        std::unordered_map<std::string, double> Tref, Pref;
        for (const auto& r : rows) {
            Tref[r.case_id] = r.hover_T_N;
            Pref[r.case_id] = r.hover_P_W;
        }

        CfdPipelineConfig cfg;
        cfg.manifest_id = "cfd_run_001";
        cfg.created_utc_iso8601 = "2026-01-03T00:00:00Z";
        cfg.notes = "Top-N promotion for interference validation";
        cfg.selection.top_n = 2;
        cfg.selection.require_go = true;
        cfg.selection.sort_by_lowest_hover_power = true;
        cfg.selection.tier = CfdTier::CFD0_ActuatorDisk;

        // Thresholds used only if recompute_gonogo = true
        cfg.thresholds.delta_mass_max_kg = 0.0;
        cfg.thresholds.A_total_min_m2 = 0.0;
        cfg.thresholds.disk_loading_max_N_m2 = 0.0;
        cfg.thresholds.hover_power_max_W = 0.0;
        cfg.thresholds.fm_min = 0.0;

        // Placeholder: external CFD results CSV (empty => manifest only)
        const std::string cfd_results_csv = "";

        const auto out = run_cfd_pipeline(rows, gonogo, Tref, Pref, cfd_results_csv, cfg);

        std::cout << "===== cfd_manifest.json =====\n";
        std::cout << out.manifest_json << "\n\n";

        std::cout << "===== cfd_manifest.csv =====\n";
        std::cout << out.manifest_csv << "\n\n";

        if (!cfd_results_csv.empty()) {
            std::cout << "===== closeout_corrected.csv =====\n";
            std::cout << out.corrected_closeout_csv << "\n\n";

            std::cout << "===== gonogo_corrected.csv =====\n";
            std::cout << out.corrected_gonogo_csv << "\n\n";
        }

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
