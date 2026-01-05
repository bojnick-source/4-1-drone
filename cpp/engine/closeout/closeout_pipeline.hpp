/*
===============================================================================
Fragment 3.1.46 — Closeout Pipeline Orchestrator (Evidence Builder + Gate Eval + Risk/CDF Hooks) (C++)
File: cpp/engine/closeout/closeout_pipeline.hpp
===============================================================================
*/

#pragma once
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include "../closeout/go_nogo_thresholds.hpp"
#include "../compliance/rules_verification.hpp"
#include "../stats/empirical_cdf.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

namespace lift::closeout {

// -----------------------------
// Export-friendly key/value record (no JSON dependency)
// -----------------------------
struct KV final {
    std::string key;
    double value = 0.0;
    std::string unit;
    std::string source;

    void validate() const {
        LIFT_BEMT_REQUIRE(!key.empty(), lift::bemt::ErrorCode::InvalidInput, "KV.key empty");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(value), lift::bemt::ErrorCode::InvalidInput, "KV.value not finite");
    }
};

// -----------------------------
// Closeout inputs: “already computed” metrics + optional distributions
// -----------------------------
struct CloseoutInputs final {
    // Core mass + rotor metrics
    double d_mass_kg = 0.0;
    double mass_empty_kg = 0.0;

    double A_total_m2 = 0.0;
    double disk_loading_N_m2 = 0.0;
    double P_hover_1g_W = 0.0;

    // Drag computed at target V (from physics/parasite_drag compare)
    // Provide these if you already computed; pipeline can still gate them via GateInputs.
    double CdS_total_m2 = 0.0;
    double P_parasite_at_V_W = 0.0;
    double V_drag_target_mps = 0.0;

    // Maneuverability (from controls/maneuverability)
    lift::controls::ManeuverMetrics maneuver{};

    // Optional subreports
    bool has_sync = false;
    lift::propulsion::SyncEvalOut sync{};

    bool has_struct = false;
    lift::structures::GearboxFeasibilityOut struct_out{};

    bool has_mission = false;
    lift::mission::MissionResult mission{};

    bool has_sfcs = false;
    lift::integration::Report sfcs{};

    // Compliance: clauses + evidence are evaluated here (optional)
    bool has_compliance_clauses = false;
    std::vector<lift::compliance::Clause> clauses;

    // Optional additional evidence items from upstream systems (mass ledger details, CG, inertia, etc.)
    std::vector<lift::compliance::EvidenceItem> extra_evidence;

    // Optional statistical distributions for risk reporting
    // Convention: Each distribution ECDF corresponds to a numeric metric id.
    std::vector<std::pair<std::string, lift::stats::EmpiricalCDF>> distributions;

    void validate() const {
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(d_mass_kg), lift::bemt::ErrorCode::InvalidInput, "d_mass invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(mass_empty_kg) && mass_empty_kg >= 0.0, lift::bemt::ErrorCode::InvalidInput, "mass_empty invalid");

        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(A_total_m2) && A_total_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "A_total invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(disk_loading_N_m2) && disk_loading_N_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "disk_loading invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_hover_1g_W) && P_hover_1g_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "P_hover invalid");

        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(CdS_total_m2) && CdS_total_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "CdS invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_parasite_at_V_W) && P_parasite_at_V_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "P_parasite invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(V_drag_target_mps) && V_drag_target_mps >= 0.0, lift::bemt::ErrorCode::InvalidInput, "V_drag_target invalid");

        // Extra evidence validity is checked when merged.
    }
};

// -----------------------------
// Closeout configuration: thresholds + risk queries
// -----------------------------
struct RiskQuery final {
    std::string metric_id;   // must match distribution metric id
    std::string comparator;  // "<=", "<", ">=", ">"
    double threshold = 0.0;

    void validate() const {
        LIFT_BEMT_REQUIRE(!metric_id.empty(), lift::bemt::ErrorCode::InvalidConfig, "RiskQuery.metric_id empty");
        LIFT_BEMT_REQUIRE(!comparator.empty(), lift::bemt::ErrorCode::InvalidConfig, "RiskQuery.comparator empty");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(threshold), lift::bemt::ErrorCode::InvalidConfig, "RiskQuery.threshold invalid");
    }
};

struct CloseoutConfig final {
    // Numerical GO/NO-GO policy
    Thresholds thresholds{};

    // Compliance: if true, fail gate if compliance fails
    bool require_compliance_ok = false;

    // Risk reporting: list of probability queries against metric distributions
    std::vector<RiskQuery> risk_queries;

    void validate() const {
        thresholds.validate();
        for (const auto& rq : risk_queries) rq.validate();
    }
};

// -----------------------------
// Outputs
// -----------------------------
struct CloseoutOutput final {
    // Evidence and KV export
    std::vector<lift::compliance::EvidenceItem> evidence;
    std::vector<KV> export_kv;

    // Compliance report (if clauses provided)
    bool has_compliance = false;
    lift::compliance::ComplianceReport compliance{};

    // Gate report (always produced)
    GateReport gate{};

    // Risk items (optional)
    std::vector<lift::stats::RiskItem> risk_items;
};

// -----------------------------
// Helpers: add evidence safely
// -----------------------------
inline void add_evidence(std::vector<lift::compliance::EvidenceItem>& ev,
                         const std::string& key, double value,
                         const std::string& unit, const std::string& source) {
    if (!lift::bemt::is_finite(value)) return; // ignore invalid
    lift::compliance::EvidenceItem e;
    e.key = key;
    e.value = value;
    e.unit = unit;
    e.source = source;
    e.validate();
    ev.push_back(std::move(e));
}

inline void add_kv(std::vector<KV>& kv, const std::string& key, double value,
                   const std::string& unit, const std::string& source) {
    if (!lift::bemt::is_finite(value)) return;
    KV r{key, value, unit, source};
    r.validate();
    kv.push_back(std::move(r));
}

// Find distribution by id (linear scan; distribution list expected small)
inline lift::stats::EmpiricalCDF* find_dist(std::vector<std::pair<std::string, lift::stats::EmpiricalCDF>>& dists,
                                            const std::string& id) noexcept {
    for (auto& p : dists) if (p.first == id) return &p.second;
    return nullptr;
}

// -----------------------------
// Main pipeline
// -----------------------------
inline CloseoutOutput run_closeout_pipeline(const CloseoutInputs& in_in, const CloseoutConfig& cfg_in) {
    CloseoutInputs in = in_in;
    in.validate();

    CloseoutConfig cfg = cfg_in;
    cfg.validate();

    CloseoutOutput out;

    // 1) Build minimal canonical evidence
    add_evidence(out.evidence, "d_mass_kg", in.d_mass_kg, "kg", "mass_ledger");
    add_evidence(out.evidence, "mass_empty_kg", in.mass_empty_kg, "kg", "mass_ledger");

    add_evidence(out.evidence, "A_total_m2", in.A_total_m2, "m^2", "geometry");
    add_evidence(out.evidence, "disk_loading_N_m2", in.disk_loading_N_m2, "N/m^2", "bemt_hover");
    add_evidence(out.evidence, "P_hover_1g_W", in.P_hover_1g_W, "W", "bemt_hover");

    add_evidence(out.evidence, "CdS_total_m2", in.CdS_total_m2, "m^2", "parasite_drag");
    add_evidence(out.evidence, "P_parasite_at_V_W", in.P_parasite_at_V_W, "W", "parasite_drag");
    add_evidence(out.evidence, "V_drag_target_mps", in.V_drag_target_mps, "m/s", "parasite_drag");

    add_evidence(out.evidence, "yaw_margin", in.maneuver.yaw_margin, "-", "maneuver");
    add_evidence(out.evidence, "roll_margin", in.maneuver.roll_margin, "-", "maneuver");
    add_evidence(out.evidence, "pitch_margin", in.maneuver.pitch_margin, "-", "maneuver");

    add_evidence(out.evidence, "yaw_alpha_max", in.maneuver.yaw_alpha_max, "rad/s^2", "maneuver");
    add_evidence(out.evidence, "roll_alpha_max", in.maneuver.roll_alpha_max, "rad/s^2", "maneuver");
    add_evidence(out.evidence, "pitch_alpha_max", in.maneuver.pitch_alpha_max, "rad/s^2", "maneuver");
    add_evidence(out.evidence, "turn_radius_m", in.maneuver.turn_radius_m, "m", "maneuver");

    if (in.has_sync) {
        add_evidence(out.evidence, "sync_margin", in.sync.metrics.margin, "-", "sync_eval");
    }
    if (in.has_struct) {
        add_evidence(out.evidence, "mast_mass_kg", in.struct_out.metrics.mast_mass_kg, "kg", "struct_eval");
        add_evidence(out.evidence, "gearbox_housing_mass_kg", in.struct_out.metrics.housing_mass_kg, "kg", "struct_eval");
        add_evidence(out.evidence, "gearbox_backlash_deg", in.struct_out.metrics.backlash_deg, "deg", "struct_eval");
    }
    if (in.has_mission) {
        add_evidence(out.evidence, "mission_total_time_s", in.mission.total_time_s, "s", "mission");
        add_evidence(out.evidence, "mission_total_energy_J", in.mission.total_energy_J, "J", "mission");
        add_evidence(out.evidence, "mission_score", in.mission.score, "-", "mission");
    }

    // 2) Merge extra evidence (dedupe by key: last wins)
    // Keep deterministic: append then allow compliance map to choose last by unordered_map policy.
    for (const auto& e : in.extra_evidence) {
        // Validate but do not hard-fail if user passes empty list; required fields enforce.
        e.validate();
        out.evidence.push_back(e);
    }

    // 3) Export KV mirror (for simple CSV/log dump)
    out.export_kv.reserve(out.evidence.size());
    for (const auto& e : out.evidence) {
        add_kv(out.export_kv, e.key, e.value, e.unit, e.source);
    }

    // 4) Evaluate compliance if clauses provided
    if (in.has_compliance_clauses && !in.clauses.empty()) {
        out.has_compliance = true;
        out.compliance = lift::compliance::evaluate_compliance(in.clauses, out.evidence);
    }

    // 5) Gate evaluation
    // Build GateInputs using already-computed values. (No recompute here.)
    GateInputs gi;
    gi.d_mass_kg = in.d_mass_kg;
    gi.mass_empty_kg = in.mass_empty_kg;

    gi.A_total_m2 = in.A_total_m2;
    gi.disk_loading_N_m2 = in.disk_loading_N_m2;
    gi.P_hover_1g_W = in.P_hover_1g_W;

    // Drag gating: we inject candidate drag directly via synthetic single-item vectors
    // to avoid requiring baseline/candidate breakdown here.
    // If you want per-item CdS, pass those vectors directly into GateInputs elsewhere.
    gi.atm = lift::physics::Atmosphere{}; // defaults validated by GateInputs.validate in go/no-go, but we bypass drag compare here.
    gi.baseline_drag_items.clear();
    gi.candidate_drag_items.clear();
    // Instead of drag compare, go/no-go uses compare_drag if V and P_parasite threshold enabled.
    // To keep this pipeline generic, we set V in thresholds and directly gate CdS/P via evidence-driven thresholds, not compare_drag.
    // Therefore: we disable internal drag compare by setting V_drag_target_mps to 0 in thresholds if needed.
    // (If you want compare_drag path, populate baseline/candidate vectors at call site.)

    gi.maneuver = in.maneuver;

    gi.has_sync = in.has_sync;
    gi.sync = in.sync;

    gi.has_struct = in.has_struct;
    gi.struct_out = in.struct_out;

    gi.has_mission = in.has_mission;
    gi.mission = in.mission;

    gi.has_sfcs = in.has_sfcs;
    gi.sfcs = in.sfcs;

    gi.has_compliance = out.has_compliance;
    gi.compliance = out.compliance;

    // IMPORTANT: GateInputs.validate requires atmosphere valid; default atmosphere is valid.
    // Drag compare inside evaluate_go_nogo requires V and P_parasite thresholds enabled; we recommend gating CdS/P via evidence.
    // So we set thresholds V_drag_target_mps = 0.0 to avoid compare_drag in this pipeline path.
    Thresholds thr = cfg.thresholds;
    // If caller provided explicit CdS/P values and wants gating, do it using additional checks below.
    // Keep compare_drag disabled unless caller supplies per-item drag tables.
    thr.V_drag_target_mps = 0.0;

    // Force compliance requirement into thresholds-based gating behavior:
    thr.require_compliance_ok = cfg.require_compliance_ok;

    out.gate = evaluate_go_nogo(gi, thr);

    // Add direct CdS/P_parasite gates (since compare_drag disabled here)
    // These are deterministic and use CloseoutInputs fields.
    if (cfg.thresholds.CdS_max_m2 > 0.0) {
        const bool pass = (in.CdS_total_m2 <= cfg.thresholds.CdS_max_m2);
        out.gate.checks.push_back({"GATE.DRAG.CDS_MAX_M2_DIRECT", pass, in.CdS_total_m2, cfg.thresholds.CdS_max_m2,
                                   pass ? "" : "CdS exceeds max"});
        if (!pass) out.gate.verdict = Verdict::NoGo;
    }
    if (cfg.thresholds.P_parasite_max_W > 0.0) {
        const bool pass = (in.P_parasite_at_V_W <= cfg.thresholds.P_parasite_max_W);
        out.gate.checks.push_back({"GATE.DRAG.P_PARASITE_MAX_W_DIRECT", pass, in.P_parasite_at_V_W, cfg.thresholds.P_parasite_max_W,
                                   pass ? "" : "P_parasite exceeds max"});
        if (!pass) out.gate.verdict = Verdict::NoGo;
    }

    // If compliance required by config, enforce it even if thresholds disabled
    if (cfg.require_compliance_ok) {
        const bool pass = out.has_compliance && out.compliance.ok();
        out.gate.checks.push_back({"GATE.COMPLIANCE.OK_PIPELINE", pass, pass ? 1.0 : 0.0, 1.0,
                                   pass ? "" : "compliance required and not OK"});
        if (!pass) out.gate.verdict = Verdict::NoGo;
    }

    // 6) Risk reporting from distributions
    if (!cfg.risk_queries.empty() && !in.distributions.empty()) {
        // Make a mutable copy because ECDF methods finalize/sort internally.
        auto dists = in.distributions;

        out.risk_items.reserve(cfg.risk_queries.size());
        for (const auto& rq : cfg.risk_queries) {
            auto* ecdf = find_dist(dists, rq.metric_id);
            if (!ecdf) continue;

            lift::stats::RiskItem ri = lift::stats::build_risk_item(rq.metric_id, rq.comparator, rq.threshold, *ecdf);
            out.risk_items.push_back(std::move(ri));
        }
    }

    return out;
}

} // namespace lift::closeout

