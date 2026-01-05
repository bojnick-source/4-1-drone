/*
===============================================================================
Fragment 3.2.12 — CFD Pipeline With Audit Tags (Emit schema+hash for every artifact) (C++)
File: cfd_pipeline_audited.cpp
===============================================================================
*/

#include "cfd_pipeline_audited.hpp"

namespace lift::bemt {

CfdPipelineAuditedOutputs run_cfd_pipeline_audited(const std::vector<CloseoutRow>& closeout_rows,
                                                   const std::vector<GoNoGoReport>& gonogo_reports,
                                                   const std::unordered_map<std::string, double>& bemt_T_ref,
                                                   const std::unordered_map<std::string, double>& bemt_P_ref,
                                                   const std::string& cfd_results_csv,
                                                   const CfdPipelineGatedConfig& cfg) {
    // Run gated pipeline first.
    CfdPipelineAuditedOutputs out{};
    out.gated = run_cfd_pipeline_gated(closeout_rows, gonogo_reports, bemt_T_ref, bemt_P_ref, cfd_results_csv, cfg);

    // Always audit manifest outputs.
    out.manifest_json_audit = ::lift::bemt::cfd::audit_manifest_json(out.gated.base.manifest_json);
    out.manifest_csv_audit  = ::lift::bemt::cfd::audit_manifest_csv(out.gated.base.manifest_csv);

    // Optional: CFD results (only if provided).
    if (!cfd_results_csv.empty()) {
        out.has_results_audit = true;
        out.results_csv_audit = ::lift::bemt::cfd::audit_results_csv(cfd_results_csv);
    }

    // Optional: corrected outputs (only when calibration applied and non-empty).
    if (out.gated.calibration_enabled && !out.gated.base.corrected_closeout_csv.empty()) {
        out.has_corrected_audit = true;
        out.corrected_closeout_audit = ::lift::bemt::cfd::audit_corrected_closeout_csv(out.gated.base.corrected_closeout_csv);
        out.corrected_gonogo_audit   = ::lift::bemt::cfd::audit_corrected_gonogo_csv(out.gated.base.corrected_gonogo_csv);
    }

    return out;
}

} // namespace lift::bemt
