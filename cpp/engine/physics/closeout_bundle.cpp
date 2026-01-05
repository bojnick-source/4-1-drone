// ============================================================================
// Fragment 3.4.02 — Closeout Bundle Bugfix (Correct CFD audited access + corrected artifacts) (C++)
// File: closeout_bundle.cpp
// ============================================================================
//
// Fixes:
// - Correct member access for CfdPipelineAuditedOutputs:
//     cfd_opt->gated (CfdPipelineGatedOutputs)
//     cfd_opt->gated.base (CfdPipelineOutputs)
// - Corrected calibration_enabled / corrected CSV retrieval.
// ============================================================================

#include "closeout_bundle.hpp"
#include "prob_closeout_csv.hpp" // prob_gate_csv helpers
#include "cdf_report_csv.hpp"    // prob_csv helpers
#include "cfd_audit.hpp"         // CFD schema constants + hash helpers

namespace lift::bemt {

namespace {

void append_tag(std::string& dst, const std::string& tag) {
    dst.append(tag);
    dst.push_back('\n');
}

BundleAudit to_bundle_audit(const ::lift::bemt::cfd::ArtifactAudit& a) {
    BundleAudit b;
    b.schema = a.schema;
    b.hash_hex = a.hash_hex;
    b.tag = a.tag;
    return b;
}

} // namespace

BundleAudit audit_blob(const char* schema, const std::string& content) {
    LIFT_BEMT_REQUIRE(schema && *schema, ErrorCode::InvalidInput, "audit_blob schema empty");
    const std::uint64_t h = ::lift::bemt::cfd::fnv1a64(content);
    BundleAudit a;
    a.schema = schema;
    a.hash_hex = ::lift::bemt::cfd::hex64(h);
    a.tag = std::string(schema) + ":" + a.hash_hex;
    return a;
}

BundleAudit audit_bundle(const CloseoutBundleAuditTags& tags) {
    // Fixed order — do not change without bumping bundle schema.
    std::string cat;
    cat.reserve(512);

    append_tag(cat, tags.closeout_csv_audit.tag);
    append_tag(cat, tags.gonogo_csv_audit.tag);

    if (tags.has_prob) {
        append_tag(cat, tags.prob_closeout_csv_audit.tag);
        append_tag(cat, tags.prob_gates_csv_audit.tag);
    }
    if (tags.has_cfd) {
        append_tag(cat, tags.cfd_manifest_json_audit.tag);
        append_tag(cat, tags.cfd_manifest_csv_audit.tag);
    }
    if (tags.has_cfd_corrected) {
        append_tag(cat, tags.corrected_closeout_csv_audit.tag);
        append_tag(cat, tags.corrected_gonogo_csv_audit.tag);
    }

    return audit_blob(kCloseoutBundleSchemaVersion, cat);
}

CloseoutBundleOutputs build_closeout_bundle(const std::vector<CloseoutRow>& closeout_rows,
                                            const std::vector<GoNoGoReport>& gonogo_reports,
                                            const ::lift::bemt::prob::ProbCloseoutOutputs* prob_opt,
                                            const ::lift::bemt::CfdPipelineAuditedOutputs* cfd_opt,
                                            const CloseoutBundleConfig& cfg_in) {
    CloseoutBundleConfig cfg = cfg_in;
    cfg.validate();

    CloseoutBundleOutputs out;

    // --- Deterministic closeout + gonogo ---
    out.artifacts.closeout_csv = closeout_csv(closeout_rows);
    out.artifacts.gonogo_csv   = gonogo_csv(gonogo_reports);

    out.audits.closeout_csv_audit = audit_blob("closeout_csv_v1", out.artifacts.closeout_csv);
    out.audits.gonogo_csv_audit   = audit_blob("gonogo_csv_v1",   out.artifacts.gonogo_csv);

    // --- Probability artifacts (optional) ---
    if (cfg.include_probability && prob_opt) {
        out.artifacts.has_prob = true;
        out.artifacts.prob_closeout_csv = prob_opt->prob_closeout_csv;
        out.artifacts.prob_gates_csv    = prob_opt->prob_gates_csv;

        out.audits.has_prob = true;
        out.audits.prob_closeout_csv_audit = audit_blob("prob_closeout_csv_v1", out.artifacts.prob_closeout_csv);
        out.audits.prob_gates_csv_audit    = audit_blob("prob_gates_csv_v1",    out.artifacts.prob_gates_csv);
    }

    // --- CFD artifacts (optional) ---
    if (cfg.include_cfd && cfd_opt) {
        out.artifacts.has_cfd = true;
        out.artifacts.cfd_manifest_json = cfd_opt->gated.base.manifest_json;
        out.artifacts.cfd_manifest_csv  = cfd_opt->gated.base.manifest_csv;

        out.audits.has_cfd = true;
        out.audits.cfd_manifest_json_audit = to_bundle_audit(cfd_opt->manifest_json_audit);
        out.audits.cfd_manifest_csv_audit  = to_bundle_audit(cfd_opt->manifest_csv_audit);

        const bool corrected_available =
            cfd_opt->gated.calibration_enabled &&
            cfd_opt->has_corrected_audit &&
            !cfd_opt->gated.base.corrected_closeout_csv.empty() &&
            !cfd_opt->gated.base.corrected_gonogo_csv.empty();

        if (cfg.include_cfd_corrected && corrected_available) {
            out.artifacts.has_cfd_corrected = true;
            out.artifacts.corrected_closeout_csv = cfd_opt->gated.base.corrected_closeout_csv;
            out.artifacts.corrected_gonogo_csv   = cfd_opt->gated.base.corrected_gonogo_csv;

            out.audits.has_cfd_corrected = true;
            out.audits.corrected_closeout_csv_audit = to_bundle_audit(cfd_opt->corrected_closeout_audit);
            out.audits.corrected_gonogo_csv_audit   = to_bundle_audit(cfd_opt->corrected_gonogo_audit);
        }
    }

    // --- Bundle audit (combined) ---
    out.audits.bundle_audit = audit_bundle(out.audits);

    return out;
}

} // namespace lift::bemt
