/*
===============================================================================
Fragment 3.2.11 — Attach Audit Tags to Outputs (Manifest/Corrected/Results) (C++)
File: cfd_audit.hpp
===============================================================================
*/

#pragma once
#include "cfd_schema.hpp"

#include <string>

namespace lift::bemt::cfd {

struct ArtifactAudit final {
    std::string schema;
    std::string hash_hex;   // 16 hex chars
    std::string tag;        // "<schema>:<hash_hex>"
};

ArtifactAudit audit_manifest_json(const std::string& content);
ArtifactAudit audit_manifest_csv(const std::string& content);
ArtifactAudit audit_results_csv(const std::string& content);
ArtifactAudit audit_corrected_closeout_csv(const std::string& content);
ArtifactAudit audit_corrected_gonogo_csv(const std::string& content);

} // namespace lift::bemt::cfd
