// ============================================================================
// Fragment 3.4.01 — Closeout Bundle (Deterministic Closeout + GO/NO-GO + Optional CFD/Prob + Audit Tags)
// File: closeout_bundle.hpp
// ============================================================================
//
// Purpose:
// - Single “closeout bundle” output for a run:
//    - closeout.csv (deterministic BEMT rows)
//    - gonogo.csv   (deterministic GO/NO-GO)
//    - optional: prob_closeout.csv + prob_gates.csv
//    - optional: cfd_manifest.* + corrected_closeout.csv + corrected_gonogo.csv
// - Attach audit tags (schema:hash) to every artifact for reproducibility.
//
// This is glue only: it does NOT run optimizers or BEMT.
// It packages already-produced outputs into a stable record.
//
// ============================================================================

#pragma once
#include "bemt_closeout_csv.hpp"
#include "closeout_report_csv.hpp"
#include "prob_closeout_integration.hpp"
#include "cfd_pipeline_audited.hpp"
#include "cfd_schema.hpp"   // reuse fnv1a64/hex64 for non-crypto audits

#include <cstdint>
#include <string>
#include <vector>

namespace lift::bemt {

inline constexpr const char* kCloseoutBundleSchemaVersion = "closeout_bundle_v1";

struct BundleAudit final {
    std::string schema;    // schema version label
    std::string hash_hex;  // 16 hex chars
    std::string tag;       // "<schema>:<hash_hex>"
};

struct CloseoutBundleArtifacts final {
    // Deterministic closeout
    std::string closeout_csv;
    std::string gonogo_csv;

    // Optional probability closeout
    bool has_prob = false;
    std::string prob_closeout_csv;
    std::string prob_gates_csv;

    // Optional CFD audited outputs
    bool has_cfd = false;
    std::string cfd_manifest_json;
    std::string cfd_manifest_csv;

    bool has_cfd_corrected = false;
    std::string corrected_closeout_csv;
    std::string corrected_gonogo_csv;
};

struct CloseoutBundleAuditTags final {
    // Always
    BundleAudit closeout_csv_audit;
    BundleAudit gonogo_csv_audit;

    // Optional prob
    bool has_prob = false;
    BundleAudit prob_closeout_csv_audit;
    BundleAudit prob_gates_csv_audit;

    // Optional CFD
    bool has_cfd = false;
    BundleAudit cfd_manifest_json_audit;
    BundleAudit cfd_manifest_csv_audit;

    bool has_cfd_corrected = false;
    BundleAudit corrected_closeout_csv_audit;
    BundleAudit corrected_gonogo_csv_audit;

    // Bundle-level combined audit (hash over concatenated per-file tags)
    BundleAudit bundle_audit;
};

struct CloseoutBundleOutputs final {
    CloseoutBundleArtifacts artifacts;
    CloseoutBundleAuditTags audits;
};

struct CloseoutBundleConfig final {
    // Include probability artifacts if provided
    bool include_probability = true;

    // Include CFD artifacts if provided
    bool include_cfd = true;

    // Include corrected CFD artifacts only if they exist
    bool include_cfd_corrected = true;

    void validate() const {
        // No numeric fields. Reserved for future.
    }
};

// Build deterministic audits (schema+hash) for a single content blob.
BundleAudit audit_blob(const char* schema, const std::string& content);

// Compute a stable bundle audit by hashing the concatenation of all per-file audit tags
// in a fixed order.
BundleAudit audit_bundle(const CloseoutBundleAuditTags& tags);

// Create a complete bundle from already-produced CSV/JSON strings.
//
// Inputs:
// - closeout_rows + gonogo_reports for deterministic outputs
// - optional prob outputs (from run_probability_closeout)
// - optional CFD audited outputs (from run_cfd_pipeline_audited)
//
// Any optional inputs may be null/empty; config controls which are included.
CloseoutBundleOutputs build_closeout_bundle(const std::vector<CloseoutRow>& closeout_rows,
                                            const std::vector<GoNoGoReport>& gonogo_reports,
                                            const ::lift::bemt::prob::ProbCloseoutOutputs* prob_opt,
                                            const ::lift::bemt::CfdPipelineAuditedOutputs* cfd_opt,
                                            const CloseoutBundleConfig& cfg);

} // namespace lift::bemt
