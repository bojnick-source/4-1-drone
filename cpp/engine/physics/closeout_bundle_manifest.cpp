// ============================================================================
// Fragment 3.4.04 — Closeout Bundle Schema + Export Manifest (files, audit tags, sizes)
// File: closeout_bundle_manifest.cpp
// ============================================================================

#include "closeout_bundle_manifest.hpp"
#include "bemt_require.hpp"

#include <sstream>

namespace lift::bemt {

namespace {

BundleFileEntry mk(const std::string& name, const std::string& content, const std::string& audit_tag) {
    BundleFileEntry e;
    e.name = name;
    e.bytes = content.size();
    e.audit_tag = audit_tag;
    return e;
}

std::string jesc(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 16);
    for (char c : s) {
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"':  out += "\\\""; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default: out.push_back(c); break;
        }
    }
    return out;
}

void json_entry(std::ostringstream& os, const char* key, const BundleFileEntry& e, bool comma=true) {
    os << "    \"" << key << "\": {\n"
       << "      \"name\": \"" << jesc(e.name) << "\",\n"
       << "      \"bytes\": " << e.bytes << ",\n"
       << "      \"audit\": \"" << jesc(e.audit_tag) << "\"\n"
       << "    }" << (comma ? "," : "") << "\n";
}

std::string esc_csv(const std::string& s) {
    bool need = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') { need = true; break; }
    }
    if (!need) return s;

    std::string out;
    out.reserve(s.size() + 8);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.append("\"\"");
        else out.push_back(c);
    }
    out.push_back('"');
    return out;
}

void csv_entry(std::ostringstream& os, const std::string& group, const BundleFileEntry& e) {
    os << esc_csv(group) << ","
       << esc_csv(e.name) << ","
       << e.bytes << ","
       << esc_csv(e.audit_tag) << "\n";
}

} // namespace

CloseoutBundleManifest build_bundle_manifest(const CloseoutBundleOutputs& b,
                                            const std::string& created_utc_iso8601,
                                            const std::string& notes) {
    LIFT_BEMT_REQUIRE(!created_utc_iso8601.empty(), ErrorCode::InvalidInput, "created_utc_iso8601 empty");

    CloseoutBundleManifest m;
    m.created_utc_iso8601 = created_utc_iso8601;
    m.notes = notes;
    m.bundle_audit_tag = b.audits.bundle_audit.tag;

    m.closeout_csv = mk("closeout.csv", b.artifacts.closeout_csv, b.audits.closeout_csv_audit.tag);
    m.gonogo_csv   = mk("gonogo.csv",   b.artifacts.gonogo_csv,   b.audits.gonogo_csv_audit.tag);

    if (b.artifacts.has_prob) {
        m.has_prob = true;
        m.prob_closeout_csv = mk("prob_closeout.csv", b.artifacts.prob_closeout_csv, b.audits.prob_closeout_csv_audit.tag);
        m.prob_gates_csv    = mk("prob_gates.csv",    b.artifacts.prob_gates_csv,    b.audits.prob_gates_csv_audit.tag);
    }

    if (b.artifacts.has_cfd) {
        m.has_cfd = true;
        m.cfd_manifest_json = mk("cfd_manifest.json", b.artifacts.cfd_manifest_json, b.audits.cfd_manifest_json_audit.tag);
        m.cfd_manifest_csv  = mk("cfd_manifest.csv",  b.artifacts.cfd_manifest_csv,  b.audits.cfd_manifest_csv_audit.tag);
    }

    if (b.artifacts.has_cfd_corrected) {
        m.has_cfd_corrected = true;
        m.corrected_closeout_csv = mk("corrected_closeout.csv", b.artifacts.corrected_closeout_csv, b.audits.corrected_closeout_csv_audit.tag);
        m.corrected_gonogo_csv   = mk("corrected_gonogo.csv",   b.artifacts.corrected_gonogo_csv,   b.audits.corrected_gonogo_csv_audit.tag);
    }

    return m;
}

std::string bundle_manifest_json(const CloseoutBundleManifest& m) {
    std::ostringstream os;
    os << "{\n";
    os << "  \"schema\": \"" << jesc(m.schema) << "\",\n";
    os << "  \"bundle_audit\": \"" << jesc(m.bundle_audit_tag) << "\",\n";
    os << "  \"created_utc\": \"" << jesc(m.created_utc_iso8601) << "\",\n";
    os << "  \"notes\": \"" << jesc(m.notes) << "\",\n";
    os << "  \"files\": {\n";

    json_entry(os, "closeout_csv", m.closeout_csv);
    json_entry(os, "gonogo_csv",   m.gonogo_csv, !(m.has_prob || m.has_cfd || m.has_cfd_corrected));

    if (m.has_prob) {
        json_entry(os, "prob_closeout_csv", m.prob_closeout_csv);
        json_entry(os, "prob_gates_csv",    m.prob_gates_csv, !(m.has_cfd || m.has_cfd_corrected));
    }

    if (m.has_cfd) {
        json_entry(os, "cfd_manifest_json", m.cfd_manifest_json);
        json_entry(os, "cfd_manifest_csv",  m.cfd_manifest_csv, !m.has_cfd_corrected);
    }

    if (m.has_cfd_corrected) {
        json_entry(os, "corrected_closeout_csv", m.corrected_closeout_csv);
        json_entry(os, "corrected_gonogo_csv",   m.corrected_gonogo_csv, false);
    }

    os << "  }\n";
    os << "}\n";
    return os.str();
}

std::string bundle_manifest_csv(const CloseoutBundleManifest& m) {
    std::ostringstream os;
    os << "schema,bundle_audit,created_utc,notes\n";
    os << esc_csv(m.schema) << "," << esc_csv(m.bundle_audit_tag) << ","
       << esc_csv(m.created_utc_iso8601) << "," << esc_csv(m.notes) << "\n\n";

    os << "group,name,bytes,audit\n";
    csv_entry(os, "closeout", m.closeout_csv);
    csv_entry(os, "gonogo",   m.gonogo_csv);

    if (m.has_prob) {
        csv_entry(os, "prob", m.prob_closeout_csv);
        csv_entry(os, "prob", m.prob_gates_csv);
    }
    if (m.has_cfd) {
        csv_entry(os, "cfd", m.cfd_manifest_json);
        csv_entry(os, "cfd", m.cfd_manifest_csv);
    }
    if (m.has_cfd_corrected) {
        csv_entry(os, "cfd_corrected", m.corrected_closeout_csv);
        csv_entry(os, "cfd_corrected", m.corrected_gonogo_csv);
    }

    return os.str();
}

} // namespace lift::bemt
