/*
===============================================================================
Fragment 3.4.05 — Closeout Bundle Manifest CLI Demo (Emit JSON+CSV + audit tags) (C++)
File: closeout_bundle_manifest_demo_main.cpp
===============================================================================

Demonstrates:
- Build closeout bundle (using prior demo)
- Build bundle manifest
- Print manifest JSON + CSV

Replace stubs with real pipeline outputs.
===============================================================================
*/

#include "engine/physics/bemt_all.hpp"
#include "engine/physics/closeout_bundle.hpp"
#include "engine/physics/closeout_bundle_manifest.hpp"

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

    return {a};
}

std::vector<GoNoGoReport> demo_gonogo_reports() {
    GoNoGoReport a;
    a.case_id = "caseA";
    a.status = GoNoGoStatus::Go;
    return {a};
}

} // namespace

int main() {
    try {
        const auto closeout_rows = demo_closeout_rows();
        const auto gonogo = demo_gonogo_reports();

        // Build a minimal bundle (no prob, no CFD)
        CloseoutBundleConfig cfg;
        cfg.include_probability = false;
        cfg.include_cfd = false;
        cfg.include_cfd_corrected = false;

        const auto bundle = build_closeout_bundle(closeout_rows, gonogo, nullptr, nullptr, cfg);

        const auto man = build_bundle_manifest(bundle, "2026-01-04T00:00:00Z", "demo manifest");

        std::cout << "===== bundle_manifest.json =====\n";
        std::cout << bundle_manifest_json(man) << "\n";

        std::cout << "===== bundle_manifest.csv =====\n";
        std::cout << bundle_manifest_csv(man) << "\n";

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
