/*
===============================================================================
Fragment 3.2.06 — CFD Pipeline Wiring (Manifest + Optional Corrections)
File: cfd_pipeline.cpp
===============================================================================
*/

#include "cfd_pipeline.hpp"

namespace lift::bemt {

namespace {
std::vector<GoNoGoReport> recompute_gonogo(const std::vector<CloseoutRow>& rows,
                                           const GoNoGoThresholds& t) {
    GoNoGoEvaluator eval(t);
    return evaluate_all(rows, eval);
}

std::vector<GoNoGoReport> recompute_gonogo_corrected(const std::vector<CloseoutRowCorrected>& rows,
                                                     const GoNoGoThresholds& t) {
    std::vector<CloseoutRow> adjusted;
    adjusted.reserve(rows.size());
    for (const auto& r : rows) {
        CloseoutRow base = static_cast<const CloseoutRow&>(r);
        // Overwrite with corrected values for gating.
        base.hover_T_N = r.corr_hover_T_N;
        base.hover_P_W = r.corr_hover_P_W;
        base.fwd_T_N = r.corr_fwd_T_N;
        base.fwd_P_W = r.corr_fwd_P_W;
        adjusted.push_back(base);
    }
    return recompute_gonogo(adjusted, t);
}
} // namespace

CfdPipelineOutputs run_cfd_pipeline(
    const std::vector<CloseoutRow>& closeout_rows,
    const std::vector<GoNoGoReport>& gonogo_reports,
    const std::unordered_map<std::string, double>& bemt_T_ref,
    const std::unordered_map<std::string, double>& bemt_P_ref,
    const std::string& cfd_results_csv,
    const CfdPipelineConfig& cfg_in) {
    CfdPipelineConfig cfg = cfg_in;
    cfg.validate();

    CfdPipelineOutputs out{};

    // GO/NO-GO source
    std::vector<GoNoGoReport> gonogo_base =
        (cfg.recompute_gonogo ? recompute_gonogo(closeout_rows, cfg.thresholds) : gonogo_reports);

    // Manifest
    out.manifest = build_cfd_manifest(cfg.manifest_id,
                                      cfg.created_utc_iso8601,
                                      cfg.notes,
                                      closeout_rows,
                                      gonogo_base,
                                      cfg.selection,
                                      cfg.geometry_ref_prefix);
    out.manifest_json = cfd_manifest_json(out.manifest);
    out.manifest_csv = cfd_manifest_csv(out.manifest);

    if (!cfd_results_csv.empty()) {
        // Ingest CFD results
        out.calibration = ingest_cfd_results_csv(cfd_results_csv, bemt_T_ref, bemt_P_ref, cfg.ingest_cfg);

        CfdCalibrationTable calib_for_apply = out.calibration;

        if (cfg.apply_gates) {
            out.gate_result = gate_cfd_calibration(out.calibration, cfg.gate_thresholds);
            calib_for_apply.entries = out.gate_result.accepted;
            calib_for_apply.rebuild_index();
        } else {
            out.gate_result.code = ErrorCode::Ok;
            out.gate_result.message = "Gating disabled";
            out.gate_result.total = out.calibration.entries.size();
            out.gate_result.ok = out.calibration.entries.size();
        }

        // Apply corrections
        out.corrected_rows = apply_cfd_calibration(closeout_rows, calib_for_apply);
        out.corrected_closeout_csv = closeout_corrected_csv(out.corrected_rows);

        // Optional corrected GO/NO-GO (using corrected values)
        out.corrected_gonogo = cfg.recompute_gonogo
            ? recompute_gonogo_corrected(out.corrected_rows, cfg.thresholds)
            : gonogo_base;
        out.corrected_gonogo_csv = gonogo_csv(out.corrected_gonogo);
    }

    return out;
}

} // namespace lift::bemt
