/*
===============================================================================
Fragment 3.2.13 — CFD Audited Pipeline CLI (Print audit tags + artifacts) (C++)
File: cfd_cli_pipeline_audited_main.cpp
===============================================================================

Demonstrates:
- gated pipeline
- audit tags for manifest/results/corrected artifacts

Replace hardcoded CSV strings with real I/O later.
===============================================================================
*/

#include "engine/physics/bemt_all.hpp"
#include "engine/physics/cfd_pipeline_audited.hpp"

#include <iostream>
#include <unordered_map>
#include <vector>

using namespace lift::bemt;

namespace {

static std::vector<CloseoutRow> demo_rows() {
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

        std::unordered_map<std::string, double> Tref, Pref;
        for (const auto& r : rows) {
            Tref[r.case_id] = r.hover_T_N;
            Pref[r.case_id] = r.hover_P_W;
        }

        CfdPipelineGatedConfig cfg;

        cfg.pipeline.manifest_id = "cfd_run_003";
        cfg.pipeline.created_utc_iso8601 = "2026-01-03T00:00:00Z";
        cfg.pipeline.notes = "Audited gated calibration demo";
        cfg.pipeline.selection.top_n = 2;
        cfg.pipeline.selection.require_go = true;
        cfg.pipeline.selection.sort_by_lowest_hover_power = true;
        cfg.pipeline.selection.tier = CfdTier::CFD0_ActuatorDisk;

        cfg.pipeline.thresholds.delta_mass_max_kg = 0.0;
        cfg.pipeline.thresholds.A_total_min_m2 = 0.0;
        cfg.pipeline.thresholds.disk_loading_max_N_m2 = 0.0;
        cfg.pipeline.thresholds.hover_power_max_W = 0.0;
        cfg.pipeline.thresholds.fm_min = 0.0;

        cfg.pipeline.recompute_gonogo = true;

        cfg.gates.min_ok_cases = 1;
        cfg.gates.max_rel_err_thrust = 0.25;
        cfg.gates.max_rel_err_power  = 0.30;

        const std::string cfd_results =
            "case_id,T_cfd_N,P_cfd_W\n"
            "caseA,1470.0,46500.0\n"
            "caseB,1510.0,51000.0\n";

        const auto out = run_cfd_pipeline_audited(rows, gonogo, Tref, Pref, cfd_results, cfg);

        std::cout << "===== AUDIT TAGS =====\n";
        std::cout << "manifest_json: " << out.manifest_json_audit.tag << "\n";
        std::cout << "manifest_csv : " << out.manifest_csv_audit.tag  << "\n";
        if (out.has_results_audit) {
            std::cout << "results_csv  : " << out.results_csv_audit.tag << "\n";
        }
        if (out.has_corrected_audit) {
            std::cout << "corrected_closeout: " << out.corrected_closeout_audit.tag << "\n";
            std::cout << "corrected_gonogo : " << out.corrected_gonogo_audit.tag   << "\n";
        }
        std::cout << "\n";

        std::cout << "===== cfd_manifest.json =====\n" << out.gated.base.manifest_json << "\n\n";
        std::cout << "===== cfd_manifest.csv =====\n"  << out.gated.base.manifest_csv  << "\n\n";

        std::cout << "===== cfd_gate_summary =====\n";
        std::cout << "code=" << static_cast<unsigned>(out.gated.gate_result.code)
                  << " msg=" << out.gated.gate_result.message
                  << " total=" << out.gated.gate_result.total
                  << " ok=" << out.gated.gate_result.ok
                  << " rejected=" << out.gated.gate_result.rejected
                  << " calibration_enabled=" << (out.gated.calibration_enabled ? "true" : "false")
                  << "\n\n";

        if (out.gated.calibration_enabled) {
            std::cout << "===== closeout_corrected.csv =====\n" << out.gated.base.corrected_closeout_csv << "\n\n";
            std::cout << "===== gonogo_corrected.csv =====\n"   << out.gated.base.corrected_gonogo_csv << "\n\n";
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
