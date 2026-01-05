// ============================================================================
// Fragment 3.4.04 — Closeout Bundle Schema + Export Manifest (files, audit tags, sizes)
// File: closeout_bundle_manifest.hpp
// ============================================================================
//
// Purpose:
// - Emit a single manifest (JSON + CSV) describing which closeout artifacts exist,
//   their audit tags, and content sizes.
// - This is the “index file” you can attach to every run directory.
//
// ============================================================================

#pragma once
#include "closeout_bundle.hpp"

#include <cstddef>
#include <string>

namespace lift::bemt {

inline constexpr const char* kCloseoutBundleManifestSchemaVersion = "closeout_bundle_manifest_v1";

struct BundleFileEntry final {
    std::string name;     // e.g., "closeout.csv"
    std::size_t bytes = 0;
    std::string audit_tag; // "<schema>:<hash>"
};

struct CloseoutBundleManifest final {
    std::string schema = kCloseoutBundleManifestSchemaVersion;
    std::string bundle_audit_tag; // bundle-level audit
    std::string created_utc_iso8601; // caller supplies
    std::string notes;

    BundleFileEntry closeout_csv;
    BundleFileEntry gonogo_csv;

    bool has_prob = false;
    BundleFileEntry prob_closeout_csv;
    BundleFileEntry prob_gates_csv;

    bool has_cfd = false;
    BundleFileEntry cfd_manifest_json;
    BundleFileEntry cfd_manifest_csv;

    bool has_cfd_corrected = false;
    BundleFileEntry corrected_closeout_csv;
    BundleFileEntry corrected_gonogo_csv;
};

CloseoutBundleManifest build_bundle_manifest(const CloseoutBundleOutputs& b,
                                            const std::string& created_utc_iso8601,
                                            const std::string& notes);

std::string bundle_manifest_json(const CloseoutBundleManifest& m);
std::string bundle_manifest_csv(const CloseoutBundleManifest& m);

} // namespace lift::bemt
