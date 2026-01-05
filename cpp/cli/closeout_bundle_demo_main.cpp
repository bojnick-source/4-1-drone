/*
===============================================================================
Fragment 3.4.03 — Closeout Bundle CLI Demo (Build bundle + print audit tags) (C++)
File: closeout_bundle_demo_main.cpp
===============================================================================

Demonstrates:
- building deterministic closeout.csv + gonogo.csv from rows
- optionally attaching probability and CFD artifacts
- printing audit tags (schema:hash) and bundle audit

Replace stubs with real pipeline outputs.
===============================================================================
*/

#include "engine/physics/bemt_all.hpp"
#include "engine/physics/closeout_bundle.hpp"

#include <iostream>
#include <vector>

using namespace lift::bemt;

namespace {

std::vector<CloseoutRow> demo_closeout_rows() {
    CloseoutRow a;
    a.case_id = "caseA";
    a.hover_code = ErrorCode::Ok;
    a.hover_T_N = 1500.0;
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
    b.status = GoNoGoStatus::NoGo;

    return {a, b};
}

} // namespace

int main() {
    try {
        const auto closeout_rows = demo_closeout_rows();
        const auto gonogo = demo_gonogo_reports();

        // Optional probability outputs (stub)
        ::lift::bemt::prob::ProbCloseoutOutputs prob;
        prob.prob_closeout_csv =
            "case_id,metric,n,min,max,mean,stddev,p10,p50,p90,p95,p99,thr1,p_ge_thr1,thr2,p_ge_thr2\n"
            "caseA,hover_power_W,2000,41000,49000,45000,1200,43400,44900,46600,47200,48000,nan,nan,nan,nan\n";
        prob.prob_gates_csv =
            "case_id,pass_all,code,message,fail_keys,fail_messages,eval_count\n"
            "caseA,1,0,PASS,,,2\n";

        // Optional CFD audited outputs (stub)
        CfdPipelineAuditedOutputs cfd;
        cfd.gated.base.manifest_json =
            "{\n"
            "  \"schema\":\"cfd_manifest_v1\",\n"
            "  \"manifest_id\":\"demo\",\n"
            "  \"tier\":\"CFD0_ActuatorDisk\"\n"
            "}\n";
        cfd.gated.base.manifest_csv =
            "schema,manifest_id,tier\n"
            "cfd_manifest_v1,demo,CFD0_ActuatorDisk\n";

        // Pretend calibration enabled and corrected outputs exist
        cfd.gated.calibration_enabled = true;
        cfd.gated.base.corrected_closeout_csv =
            "case_id,A_m2,hover_T_N,hover_P_W,hover_FM,corr_hover_T_N,corr_hover_P_W,corr_hover_FM\n"
            "caseA,0.785398,1500,45000,0.65,1470,46500,0.63\n";
        cfd.gated.base.corrected_gonogo_csv =
            "case_id,status,code,reason_count,reasons\n"
            "caseA,GO,0,0,\n";

        CloseoutBundleConfig cfg;
        cfg.include_probability = true;
        cfg.include_cfd = true;
        cfg.include_cfd_corrected = true;

        const auto bundle = build_closeout_bundle(
            closeout_rows,
            gonogo,
            &prob,
            &cfd,
            cfg
        );

        // Print audit tags
        std::cout << "===== AUDIT TAGS =====\n";
        std::cout << "closeout_csv: " << bundle.audits.closeout_csv_audit.tag << "\n";
        std::cout << "gonogo_csv  : " << bundle.audits.gonogo_csv_audit.tag << "\n";

        if (bundle.audits.has_prob) {
            std::cout << "prob_closeout_csv: " << bundle.audits.prob_closeout_csv_audit.tag << "\n";
            std::cout << "prob_gates_csv   : " << bundle.audits.prob_gates_csv_audit.tag << "\n";
        }
        if (bundle.audits.has_cfd) {
            std::cout << "cfd_manifest_json: " << bundle.audits.cfd_manifest_json_audit.tag << "\n";
            std::cout << "cfd_manifest_csv : " << bundle.audits.cfd_manifest_csv_audit.tag << "\n";
        }
        if (bundle.audits.has_cfd_corrected) {
            std::cout << "corrected_closeout_csv: " << bundle.audits.corrected_closeout_csv_audit.tag << "\n";
            std::cout << "corrected_gonogo_csv  : " << bundle.audits.corrected_gonogo_csv_audit.tag << "\n";
        }

        std::cout << "bundle_audit: " << bundle.audits.bundle_audit.tag << "\n\n";

        // Print artifacts (demonstration)
        std::cout << "===== closeout.csv =====\n" << bundle.artifacts.closeout_csv << "\n";
        std::cout << "===== gonogo.csv =====\n"   << bundle.artifacts.gonogo_csv << "\n";

        if (bundle.artifacts.has_prob) {
            std::cout << "===== prob_closeout.csv =====\n" << bundle.artifacts.prob_closeout_csv << "\n";
            std::cout << "===== prob_gates.csv =====\n"    << bundle.artifacts.prob_gates_csv << "\n";
        }

        if (bundle.artifacts.has_cfd) {
            std::cout << "===== cfd_manifest.json =====\n" << bundle.artifacts.cfd_manifest_json << "\n";
            std::cout << "===== cfd_manifest.csv =====\n"  << bundle.artifacts.cfd_manifest_csv << "\n";
        }

        if (bundle.artifacts.has_cfd_corrected) {
            std::cout << "===== corrected_closeout.csv =====\n" << bundle.artifacts.corrected_closeout_csv << "\n";
            std::cout << "===== corrected_gonogo.csv =====\n"   << bundle.artifacts.corrected_gonogo_csv << "\n";
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
