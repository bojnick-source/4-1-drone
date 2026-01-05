/*
===============================================================================
Fragment 3.2.12 — CFD Pipeline With Audit Tags (Emit schema+hash for every artifact) (C++)
File: cfd_pipeline_audited.hpp
===============================================================================
*/

#pragma once
#include "cfd_pipeline_gated.hpp"
#include "cfd_audit.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace lift::bemt {

struct CfdPipelineAuditedOutputs final {
    CfdPipelineGatedOutputs gated;

    // Audit tags (always)
    ::lift::bemt::cfd::ArtifactAudit manifest_json_audit;
    ::lift::bemt::cfd::ArtifactAudit manifest_csv_audit;

    // Audit tags (optional)
    bool has_results_audit = false;
    ::lift::bemt::cfd::ArtifactAudit results_csv_audit;

    bool has_corrected_audit = false;
    ::lift::bemt::cfd::ArtifactAudit corrected_closeout_audit;
    ::lift::bemt::cfd::ArtifactAudit corrected_gonogo_audit;
};

CfdPipelineAuditedOutputs run_cfd_pipeline_audited(const std::vector<CloseoutRow>& closeout_rows,
                                                   const std::vector<GoNoGoReport>& gonogo_reports,
                                                   const std::unordered_map<std::string, double>& bemt_T_ref,
                                                   const std::unordered_map<std::string, double>& bemt_P_ref,
                                                   const std::string& cfd_results_csv,
                                                   const CfdPipelineGatedConfig& cfg);

} // namespace lift::bemt
