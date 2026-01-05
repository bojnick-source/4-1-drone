/*
===============================================================================
Fragment 3.2.06 — CFD Pipeline Wiring (Manifest + Optional Corrections)
File: cfd_pipeline.hpp
===============================================================================
*/

#pragma once

#include "cfd_manifest.hpp"
#include "cfd_results.hpp"
#include "cfd_apply.hpp"
#include "cfd_closeout_csv.hpp"
#include "cfd_gates.hpp"
#include "closeout_thresholds.hpp"
#include "closeout_report_csv.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace lift::bemt {

struct CfdPipelineConfig final {
    std::string manifest_id = "cfd_manifest";
    std::string created_utc_iso8601 = "";
    std::string notes;

    CfdSelectionPolicy selection{};
    CfdIngestConfig ingest_cfg{};
    CfdGateThresholds gate_thresholds{};
    bool apply_gates = true;

    GoNoGoThresholds thresholds{};
    bool recompute_gonogo = false;

    std::string geometry_ref_prefix = "exports/";

    void validate() const {
        selection.validate();
        ingest_cfg.validate();
        gate_thresholds.validate();
        // thresholds validated if recompute_gonogo is true.
    }
};

struct CfdPipelineOutputs final {
    CfdManifest manifest;
    std::string manifest_json;
    std::string manifest_csv;

    CfdCalibrationTable calibration;
    CfdGateResult gate_result;

    std::vector<CloseoutRowCorrected> corrected_rows;
    std::string corrected_closeout_csv;

    std::vector<GoNoGoReport> corrected_gonogo;
    std::string corrected_gonogo_csv;
};

CfdPipelineOutputs run_cfd_pipeline(
    const std::vector<CloseoutRow>& closeout_rows,
    const std::vector<GoNoGoReport>& gonogo_reports,
    const std::unordered_map<std::string, double>& bemt_T_ref,
    const std::unordered_map<std::string, double>& bemt_P_ref,
    const std::string& cfd_results_csv,
    const CfdPipelineConfig& cfg);

} // namespace lift::bemt
