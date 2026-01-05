/*
===============================================================================
Fragment 3.2.11 — Attach Audit Tags to Outputs (Manifest/Corrected/Results) (C++)
File: cfd_audit.cpp
===============================================================================
*/

#include "cfd_audit.hpp"

namespace lift::bemt::cfd {

namespace {
constexpr const char* kSchemaManifestJson = "cfd_manifest_json_v1";
constexpr const char* kSchemaManifestCsv  = "cfd_manifest_csv_v1";
constexpr const char* kSchemaResultsCsv   = "cfd_results_csv_v1";
constexpr const char* kSchemaCloseoutCsv  = "cfd_closeout_corrected_csv_v1";
constexpr const char* kSchemaGonogoCsv    = "cfd_gonogo_corrected_csv_v1";

ArtifactAudit make_audit(const char* schema, const std::string& content) {
    const std::uint64_t h = fnv1a64(content);
    const std::string hex = hex64(h);
    ArtifactAudit a;
    a.schema = schema;
    a.hash_hex = hex;
    a.tag = audit_tag(schema, content);
    return a;
}
} // namespace

ArtifactAudit audit_manifest_json(const std::string& content) {
    return make_audit(kSchemaManifestJson, content);
}

ArtifactAudit audit_manifest_csv(const std::string& content) {
    return make_audit(kSchemaManifestCsv, content);
}

ArtifactAudit audit_results_csv(const std::string& content) {
    return make_audit(kSchemaResultsCsv, content);
}

ArtifactAudit audit_corrected_closeout_csv(const std::string& content) {
    return make_audit(kSchemaCloseoutCsv, content);
}

ArtifactAudit audit_corrected_gonogo_csv(const std::string& content) {
    return make_audit(kSchemaGonogoCsv, content);
}

} // namespace lift::bemt::cfd
