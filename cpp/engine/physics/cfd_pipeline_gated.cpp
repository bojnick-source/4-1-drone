/*
===============================================================================
Fragment 3.2.08 — CFD Pipeline With Gates (Reject Bad CFD, Calibrate Only If Enough OK)
File: cfd_pipeline_gated.cpp
===============================================================================
*/

#include "cfd_pipeline_gated.hpp"

namespace lift::bemt {

namespace {
std::vector<GoNoGoReport> recompute_gonogo_corrected(const std::vector<CloseoutRowCorrected>& rows,
                                                     const GoNoGoThresholds& t) {
    std::vector<CloseoutRow> adjusted;
    adjusted.reserve(rows.size());
    for (const auto& r : rows) {
        CloseoutRow base = static_cast<const CloseoutRow&>(r);
        base.hover_T_N = r.corr_hover_T_N;
        base.hover_P_W = r.corr_hover_P_W;
        base.fwd_T_N = r.corr_fwd_T_N;
        base.fwd_P_W = r.corr_fwd_P_W;
        adjusted.push_back(base);
    }
    GoNoGoEvaluator eval(t);
    return evaluate_all(adjusted, eval);
}
} // namespace

CfdPipelineGatedOutputs run_cfd_pipeline_gated(const std::vector<CloseoutRow>& closeout_rows,
                                              const std::vector<GoNoGoReport>& gonogo_reports,
                                              const std::unordered_map<std::string, double>& bemt_T_ref,
                                              const std::unordered_map<std::string, double>& bemt_P_ref,
                                              const std::string& cfd_results_csv,
                                              const CfdPipelineGatedConfig& cfg) {
    cfg.validate();

    // Run base pipeline without gating; we'll gate here.
    CfdPipelineConfig base_cfg = cfg.pipeline;
    base_cfg.apply_gates = false;
    CfdPipelineOutputs base_out = run_cfd_pipeline(closeout_rows, gonogo_reports, bemt_T_ref, bemt_P_ref, cfd_results_csv, base_cfg);

    CfdPipelineGatedOutputs out{};
    out.base = std::move(base_out);
    out.gate_result.code = ErrorCode::NonConverged;
    out.gate_result.message = "No CFD results provided";
    out.calibration_enabled = false;

    // If no CFD data, return manifest-only.
    if (cfd_results_csv.empty() || out.base.calibration.entries.empty()) {
        return out;
    }

    // Apply gates
    out.gate_result = gate_cfd_calibration(out.base.calibration, cfg.gates);

    if (out.gate_result.code != ErrorCode::Ok) {
        // Always clear corrected outputs to avoid accidental use when gates fail.
        out.base.corrected_rows.clear();
        out.base.corrected_closeout_csv.clear();
        out.base.corrected_gonogo.clear();
        out.base.corrected_gonogo_csv.clear();

        if (!cfg.allow_manifest_only_on_gate_fail) {
            // Keep gate_result to signal failure; caller can decide whether to treat as fatal.
            return out;
        }
        return out;
    }

    // Build accepted-only calibration table
    out.accepted_calibration.entries = out.gate_result.accepted;
    out.accepted_calibration.rebuild_index();

    // Apply accepted calibration
    out.base.corrected_rows = apply_cfd_calibration(closeout_rows, out.accepted_calibration);
    out.base.corrected_closeout_csv = closeout_corrected_csv(out.base.corrected_rows);

    // GO/NO-GO recompute if requested
    if (cfg.pipeline.recompute_gonogo) {
        out.base.corrected_gonogo = recompute_gonogo_corrected(out.base.corrected_rows, cfg.pipeline.thresholds);
        out.base.corrected_gonogo_csv = gonogo_csv(out.base.corrected_gonogo);
    }

    out.calibration_enabled = true;
    return out;
}

} // namespace lift::bemt
