/*
===============================================================================
Fragment 3.2.08 — CFD Pipeline With Gates (Reject Bad CFD, Calibrate Only If Enough OK)
File: cfd_pipeline_gated.hpp
===============================================================================
*/

#pragma once
#include "cfd_pipeline.hpp"
#include "cfd_gates.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace lift::bemt {

struct CfdPipelineGatedOutputs final {
    CfdPipelineOutputs base;

    CfdGateResult gate_result;

    // When gates pass, this is the accepted-only calibration table (indexed).
    CfdCalibrationTable accepted_calibration;

    // Convenience flags
    bool calibration_enabled = false;
};

struct CfdPipelineGatedConfig final {
    CfdPipelineConfig pipeline;

    // Gate thresholds
    CfdGateThresholds gates;

    // If true, when gates fail, still emit manifest outputs (base) but no calibration.
    bool allow_manifest_only_on_gate_fail = true;

    void validate() const {
        pipeline.validate();
        gates.validate();
    }
};

CfdPipelineGatedOutputs run_cfd_pipeline_gated(const std::vector<CloseoutRow>& closeout_rows,
                                              const std::vector<GoNoGoReport>& gonogo_reports,
                                              const std::unordered_map<std::string, double>& bemt_T_ref,
                                              const std::unordered_map<std::string, double>& bemt_P_ref,
                                              const std::string& cfd_results_csv,
                                              const CfdPipelineGatedConfig& cfg);

} // namespace lift::bemt
