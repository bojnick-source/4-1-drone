/*
===============================================================================
Fragment 3.2.01 — CFD Job Manifest Generator (Top-N Export + Schema-Stable JSON/CSV)
File: cfd_manifest.cpp
===============================================================================
*/

#include "cfd_manifest.hpp"

#include <algorithm>
#include <sstream>
#include <iomanip>

namespace lift::bemt {

static std::string esc_json(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 8);
    for (char c : s) {
        switch (c) {
            case '\"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                // Control chars -> replace with space (strict JSON escaping is heavier; keep robust)
                if (static_cast<unsigned char>(c) < 0x20) out.push_back(' ');
                else out.push_back(c);
        }
    }
    return out;
}

static std::string esc_csv(const std::string& s) {
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

static const GoNoGoReport* find_report(const std::vector<GoNoGoReport>& reps, const std::string& case_id) {
    for (const auto& r : reps) {
        if (r.case_id == case_id) return &r;
    }
    return nullptr;
}

static std::string tier_str(CfdTier t) {
    switch (t) {
        case CfdTier::CFD0_ActuatorDisk: return "CFD0_ActuatorDisk";
        case CfdTier::CFD0_ActuatorLine: return "CFD0_ActuatorLine";
        case CfdTier::CFD1_ResolvedBlades: return "CFD1_ResolvedBlades";
        default: return "Unknown";
    }
}

CfdManifest build_cfd_manifest(const std::string& manifest_id,
                               const std::string& created_utc_iso8601,
                               const std::string& notes,
                               const std::vector<CloseoutRow>& closeout_rows,
                               const std::vector<GoNoGoReport>& gonogo_reports,
                               const CfdSelectionPolicy& policy_in,
                               const std::string& geometry_ref_prefix) {
    CfdSelectionPolicy policy = policy_in;
    policy.validate();

    // Filter candidates
    struct Cand { const CloseoutRow* row; const GoNoGoReport* rep; };
    std::vector<Cand> cands;
    cands.reserve(closeout_rows.size());

    for (const auto& r : closeout_rows) {
        const GoNoGoReport* gr = find_report(gonogo_reports, r.case_id);
        const bool go_ok = (!policy.require_go) || (gr && gr->status == GoNoGoStatus::Go);
        if (!go_ok) continue;

        // Require at least valid hover outputs (even if GO policy disabled)
        if (r.hover_code != ErrorCode::Ok) continue;
        if (!is_finite(r.hover_P_W) || r.hover_P_W <= 0.0) continue;

        cands.push_back(Cand{&r, gr});
    }

    // Sort
    if (policy.sort_by_lowest_hover_power) {
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b) {
                      return a.row->hover_P_W < b.row->hover_P_W;
                  });
    } else {
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b) {
                      return a.row->hover_T_N > b.row->hover_T_N;
                  });
    }

    // Truncate top-N
    if (cands.size() > policy.top_n) cands.resize(policy.top_n);

    // Build manifest
    CfdManifest m;
    m.manifest_id = manifest_id;
    m.created_utc_iso8601 = created_utc_iso8601;
    m.notes = notes;

    m.jobs.reserve(cands.size());

    for (std::size_t i = 0; i < cands.size(); ++i) {
        const CloseoutRow& r = *cands[i].row;

        CfdJob j;
        j.case_id = r.case_id;
        j.tier = policy.tier;

        // Deterministic job_id: manifest + index + case_id
        {
            std::ostringstream os;
            os << manifest_id << "_"
               << std::setw(5) << std::setfill('0') << i
               << "_" << r.case_id;
            j.job_id = os.str();
        }

        // Convention: geometry ref points to an exported CAD per case
        // e.g. exports/<case_id>/rotor.step
        j.geometry_ref = geometry_ref_prefix + r.case_id + "/rotor.step";

        // Use hover operating point expectations only (forward can be separate CFD jobs later)
        // Note: CloseoutRow doesn't carry omega/env; those are owned by the producing layer.
        // Here we output placeholders that must be filled by the caller if they want full fidelity.
        // Still robust: zeros will be caught by validate() if you call it.
        j.omega_rad_s = 0.0;
        j.V_axial_mps = 0.0;
        j.V_inplane_mps = r.V_inplane_mps;

        j.rho = 1.225;
        j.mu = 1.81e-5;

        j.bemt_T_N = r.hover_T_N;
        j.bemt_P_W = r.hover_P_W;

        j.correction_thrust = 1.0;
        j.correction_power = 1.0;

        m.jobs.push_back(std::move(j));
    }

    // Do not auto-validate here to allow caller to fill omega/env before finalizing.
    return m;
}

std::string cfd_manifest_json(const CfdManifest& m_in) {
    CfdManifest m = m_in;
    // Validate only structural fields; jobs may be partially filled by design.
    LIFT_BEMT_REQUIRE(!m.manifest_id.empty(), ErrorCode::InvalidInput, "manifest_id empty");

    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(8);

    os << "{";
    os << "\"manifest_id\":\"" << esc_json(m.manifest_id) << "\",";
    os << "\"created_utc\":\"" << esc_json(m.created_utc_iso8601) << "\",";
    os << "\"notes\":\"" << esc_json(m.notes) << "\",";
    os << "\"jobs\":[";
    for (std::size_t i = 0; i < m.jobs.size(); ++i) {
        const auto& j = m.jobs[i];
        if (i) os << ",";
        os << "{";
        os << "\"job_id\":\"" << esc_json(j.job_id) << "\",";
        os << "\"case_id\":\"" << esc_json(j.case_id) << "\",";
        os << "\"tier\":\"" << esc_json(tier_str(j.tier)) << "\",";
        os << "\"geometry_ref\":\"" << esc_json(j.geometry_ref) << "\",";
        os << "\"mesh_ref\":\"" << esc_json(j.mesh_ref) << "\",";
        os << "\"omega_rad_s\":" << j.omega_rad_s << ",";
        os << "\"V_axial_mps\":" << j.V_axial_mps << ",";
        os << "\"V_inplane_mps\":" << j.V_inplane_mps << ",";
        os << "\"rho\":" << j.rho << ",";
        os << "\"mu\":" << j.mu << ",";
        os << "\"bemt_T_N\":" << j.bemt_T_N << ",";
        os << "\"bemt_P_W\":" << j.bemt_P_W << ",";
        os << "\"correction_thrust\":" << j.correction_thrust << ",";
        os << "\"correction_power\":" << j.correction_power;
        os << "}";
    }
    os << "]";
    os << "}";
    return os.str();
}

std::string cfd_manifest_csv(const CfdManifest& m) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(8);

    os << "manifest_id,created_utc,job_id,case_id,tier,geometry_ref,mesh_ref,omega_rad_s,V_axial_mps,V_inplane_mps,rho,mu,bemt_T_N,bemt_P_W,correction_thrust,correction_power\n";
    for (const auto& j : m.jobs) {
        os
          << esc_csv(m.manifest_id) << ","
          << esc_csv(m.created_utc_iso8601) << ","
          << esc_csv(j.job_id) << ","
          << esc_csv(j.case_id) << ","
          << esc_csv(tier_str(j.tier)) << ","
          << esc_csv(j.geometry_ref) << ","
          << esc_csv(j.mesh_ref) << ","
          << j.omega_rad_s << ","
          << j.V_axial_mps << ","
          << j.V_inplane_mps << ","
          << j.rho << ","
          << j.mu << ","
          << j.bemt_T_N << ","
          << j.bemt_P_W << ","
          << j.correction_thrust << ","
          << j.correction_power
          << "\n";
    }
    return os.str();
}

} // namespace lift::bemt
