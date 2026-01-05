/*
===============================================================================
Fragment 3.1.44 — Numerical GO/NO-GO Thresholds (Δmass, A_total, Power, CdS, Sync, etc.) + Aggregated Closeout Gate (C++)
File: cpp/engine/closeout/go_nogo_thresholds.hpp
===============================================================================
*/

#pragma once
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include "../physics/parasite_drag.hpp"
#include "../controls/maneuverability.hpp"
#include "../propulsion/sync_intermesh.hpp"
#include "../structures/gearbox_feasibility.hpp"
#include "../mission/mission_scoring.hpp"
#include "../compliance/rules_verification.hpp"
#include "../integration/sfcs_corridor_checks.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

namespace lift::closeout {

enum class Verdict : std::uint8_t { Go=0, NoGo=1, Unknown=2 };

struct GateCheck final {
    std::string id;
    bool pass = true;
    double value = 0.0;
    double threshold = 0.0;
    std::string note;

    void validate() const {
        LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "GateCheck.id empty");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(value), lift::bemt::ErrorCode::InvalidInput, "GateCheck.value invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(threshold), lift::bemt::ErrorCode::InvalidInput, "GateCheck.threshold invalid");
    }
};

struct GateReport final {
    lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
    Verdict verdict = Verdict::Unknown;
    std::vector<GateCheck> checks;

    bool ok() const noexcept { return verdict == Verdict::Go && code == lift::bemt::ErrorCode::Ok; }
};

// Policy thresholds (numerical GO/NO-GO)
struct Thresholds final {
    // Mass
    double d_mass_max_kg = 0.0;        // Δmass must be <= this (<=0 disables)
    double mass_empty_max_kg = 0.0;    // absolute empty mass gate (<=0 disables)

    // Disk area and hover power
    double A_total_min_m2 = 0.0;       // total disk area >= min (<=0 disables)
    double P_hover_1g_max_W = 0.0;     // hover power <= max (<=0 disables)
    double DL_max_N_m2 = 0.0;          // disk loading <= max (<=0 disables)

    // Parasite drag / CdS
    double CdS_max_m2 = 0.0;           // total CdS <= max (<=0 disables)
    double P_parasite_max_W = 0.0;     // at target V <= max (<=0 disables)
    double V_drag_target_mps = 0.0;    // speed used for P_parasite gate

    // Maneuverability margins
    double yaw_margin_min = 0.0;       // available/required >= min (<=0 disables)
    double roll_margin_min = 0.0;
    double pitch_margin_min = 0.0;
    double yaw_alpha_min = 0.0;        // rad/s^2 proxies (<=0 disables)
    double roll_alpha_min = 0.0;
    double pitch_alpha_min = 0.0;
    double turn_radius_max_m = 0.0;    // <=0 disables

    // Sync
    double sync_margin_min = 0.0;      // allowable/total >= min (<=0 disables)
    bool require_sync_ok = false;

    // Structures / gearbox
    bool require_struct_ok = false;

    // Mission scoring
    double mission_score_max = 0.0;    // <=0 disables
    double mission_time_max_s = 0.0;   // <=0 disables

    // Compliance
    bool require_compliance_ok = false;

    // SFCS
    bool require_sfcs_ok = false;

    void validate() const {
        // All must be finite and non-negative where applicable
        auto fin = [](double x){ return lift::bemt::is_finite(x); };
        LIFT_BEMT_REQUIRE(fin(d_mass_max_kg) && d_mass_max_kg >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "d_mass_max invalid");
        LIFT_BEMT_REQUIRE(fin(mass_empty_max_kg) && mass_empty_max_kg >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "mass_empty_max invalid");

        LIFT_BEMT_REQUIRE(fin(A_total_min_m2) && A_total_min_m2 >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "A_total_min invalid");
        LIFT_BEMT_REQUIRE(fin(P_hover_1g_max_W) && P_hover_1g_max_W >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "P_hover_max invalid");
        LIFT_BEMT_REQUIRE(fin(DL_max_N_m2) && DL_max_N_m2 >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "DL_max invalid");

        LIFT_BEMT_REQUIRE(fin(CdS_max_m2) && CdS_max_m2 >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "CdS_max invalid");
        LIFT_BEMT_REQUIRE(fin(P_parasite_max_W) && P_parasite_max_W >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "P_parasite_max invalid");
        LIFT_BEMT_REQUIRE(fin(V_drag_target_mps) && V_drag_target_mps >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "V_drag_target invalid");

        LIFT_BEMT_REQUIRE(fin(yaw_margin_min) && yaw_margin_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "yaw_margin_min invalid");
        LIFT_BEMT_REQUIRE(fin(roll_margin_min) && roll_margin_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "roll_margin_min invalid");
        LIFT_BEMT_REQUIRE(fin(pitch_margin_min) && pitch_margin_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "pitch_margin_min invalid");

        LIFT_BEMT_REQUIRE(fin(yaw_alpha_min) && yaw_alpha_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "yaw_alpha_min invalid");
        LIFT_BEMT_REQUIRE(fin(roll_alpha_min) && roll_alpha_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "roll_alpha_min invalid");
        LIFT_BEMT_REQUIRE(fin(pitch_alpha_min) && pitch_alpha_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "pitch_alpha_min invalid");

        LIFT_BEMT_REQUIRE(fin(turn_radius_max_m) && turn_radius_max_m >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "turn_radius_max invalid");
        LIFT_BEMT_REQUIRE(fin(sync_margin_min) && sync_margin_min >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "sync_margin_min invalid");

        LIFT_BEMT_REQUIRE(fin(mission_score_max) && mission_score_max >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "mission_score_max invalid");
        LIFT_BEMT_REQUIRE(fin(mission_time_max_s) && mission_time_max_s >= 0.0, lift::bemt::ErrorCode::InvalidConfig, "mission_time_max invalid");
    }
};

// Inputs assembled from other layers
struct GateInputs final {
    // Mass-related (from mass ledger / ratio calc)
    double d_mass_kg = 0.0;
    double mass_empty_kg = 0.0;

    // Disk area and hover metrics (from BEMT + geometry aggregation)
    double A_total_m2 = 0.0;
    double disk_loading_N_m2 = 0.0;
    double P_hover_1g_W = 0.0;

    // Drag model items (baseline/candidate)
    std::vector<lift::physics::DragItem> baseline_drag_items;
    std::vector<lift::physics::DragItem> candidate_drag_items;
    lift::physics::Atmosphere atm;

    // Maneuverability computed
    lift::controls::ManeuverMetrics maneuver{};

    // Sync computed
    bool has_sync = false;
    lift::propulsion::SyncEvalOut sync{};

    // Structures computed
    bool has_struct = false;
    lift::structures::GearboxFeasibilityOut struct_out{};

    // Mission result
    bool has_mission = false;
    lift::mission::MissionResult mission{};

    // Compliance report
    bool has_compliance = false;
    lift::compliance::ComplianceReport compliance{};

    // SFCS report
    bool has_sfcs = false;
    lift::integration::Report sfcs{};

    void validate() const {
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(d_mass_kg), lift::bemt::ErrorCode::InvalidInput, "d_mass invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(mass_empty_kg) && mass_empty_kg >= 0.0, lift::bemt::ErrorCode::InvalidInput, "mass_empty invalid");

        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(A_total_m2) && A_total_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "A_total invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(disk_loading_N_m2) && disk_loading_N_m2 >= 0.0, lift::bemt::ErrorCode::InvalidInput, "DL invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_hover_1g_W) && P_hover_1g_W >= 0.0, lift::bemt::ErrorCode::InvalidInput, "P_hover invalid");

        atm.validate();
    }
};

inline void add_check(std::vector<GateCheck>& v, std::string id, bool pass, double val, double thr, std::string note = {}) {
    v.push_back({std::move(id), pass, val, thr, std::move(note)});
}

// Evaluate a <= gate if thr>0 else disabled
inline void gate_leq(std::vector<GateCheck>& v, const std::string& id, double val, double thr, const std::string& note) {
    if (!lift::bemt::is_finite(val) || !lift::bemt::is_finite(thr) || thr <= 0.0) {
        add_check(v, id, true, val, thr, "disabled/invalid");
        return;
    }
    add_check(v, id, val <= thr, val, thr, (val <= thr) ? "" : note);
}
inline void gate_geq(std::vector<GateCheck>& v, const std::string& id, double val, double thr, const std::string& note) {
    if (!lift::bemt::is_finite(val) || !lift::bemt::is_finite(thr) || thr <= 0.0) {
        add_check(v, id, true, val, thr, "disabled/invalid");
        return;
    }
    add_check(v, id, val >= thr, val, thr, (val >= thr) ? "" : note);
}

// Main aggregator
inline GateReport evaluate_go_nogo(const GateInputs& in_in, const Thresholds& thr_in) {
    GateInputs in = in_in;
    in.validate();

    Thresholds thr = thr_in;
    thr.validate();

    GateReport rep;
    rep.code = lift::bemt::ErrorCode::Ok;

    // Mass gates
    gate_leq(rep.checks, "GATE.MASS.DELTA_MAX_KG", in.d_mass_kg, thr.d_mass_max_kg, "Δmass exceeds max");
    gate_leq(rep.checks, "GATE.MASS.EMPTY_MAX_KG", in.mass_empty_kg, thr.mass_empty_max_kg, "empty mass exceeds max");

    // Area/power gates
    gate_geq(rep.checks, "GATE.ROTOR.A_TOTAL_MIN_M2", in.A_total_m2, thr.A_total_min_m2, "total disk area below minimum");
    gate_leq(rep.checks, "GATE.ROTOR.DISK_LOADING_MAX", in.disk_loading_N_m2, thr.DL_max_N_m2, "disk loading exceeds max");
    gate_leq(rep.checks, "GATE.POWER.HOVER_1G_MAX_W", in.P_hover_1g_W, thr.P_hover_1g_max_W, "hover power exceeds max");

    // Drag gates
    if (thr.V_drag_target_mps > 0.0 && thr.P_parasite_max_W > 0.0) {
        const auto dd = lift::physics::compare_drag(in.baseline_drag_items, in.candidate_drag_items, in.atm, thr.V_drag_target_mps);
        gate_leq(rep.checks, "GATE.DRAG.CDS_MAX_M2", dd.cand.CdS_total_m2, thr.CdS_max_m2, "CdS exceeds max");
        gate_leq(rep.checks, "GATE.DRAG.P_PARASITE_MAX_W", dd.P_cand_W, thr.P_parasite_max_W, "parasite power exceeds max at V");
    } else {
        add_check(rep.checks, "GATE.DRAG.CDS_MAX_M2", true, 0.0, thr.CdS_max_m2, "disabled");
        add_check(rep.checks, "GATE.DRAG.P_PARASITE_MAX_W", true, 0.0, thr.P_parasite_max_W, "disabled");
    }

    // Maneuverability
    gate_geq(rep.checks, "GATE.MANEUVER.YAW_MARGIN_MIN", in.maneuver.yaw_margin, thr.yaw_margin_min, "yaw margin below minimum");
    gate_geq(rep.checks, "GATE.MANEUVER.ROLL_MARGIN_MIN", in.maneuver.roll_margin, thr.roll_margin_min, "roll margin below minimum");
    gate_geq(rep.checks, "GATE.MANEUVER.PITCH_MARGIN_MIN", in.maneuver.pitch_margin, thr.pitch_margin_min, "pitch margin below minimum");

    gate_geq(rep.checks, "GATE.MANEUVER.YAW_ALPHA_MIN", in.maneuver.yaw_alpha_max, thr.yaw_alpha_min, "yaw bandwidth proxy below minimum");
    gate_geq(rep.checks, "GATE.MANEUVER.ROLL_ALPHA_MIN", in.maneuver.roll_alpha_max, thr.roll_alpha_min, "roll bandwidth proxy below minimum");
    gate_geq(rep.checks, "GATE.MANEUVER.PITCH_ALPHA_MIN", in.maneuver.pitch_alpha_max, thr.pitch_alpha_min, "pitch bandwidth proxy below minimum");

    gate_leq(rep.checks, "GATE.MANEUVER.TURN_RADIUS_MAX_M", in.maneuver.turn_radius_m, thr.turn_radius_max_m, "turn radius exceeds max");

    // Sync
    if (thr.require_sync_ok) {
        if (!in.has_sync) {
            add_check(rep.checks, "GATE.SYNC.PRESENT", false, 0.0, 1.0, "sync required but not evaluated");
        } else {
            // metric margin: allowable/total stored as metrics.margin
            gate_geq(rep.checks, "GATE.SYNC.MARGIN_MIN", in.sync.metrics.margin, thr.sync_margin_min, "sync margin below minimum");
            add_check(rep.checks, "GATE.SYNC.REPORT_OK", in.sync.report.ok(), in.sync.report.ok() ? 1.0 : 0.0, 1.0,
                      in.sync.report.ok() ? "" : "sync report contains failing checks");
        }
    } else {
        add_check(rep.checks, "GATE.SYNC.PRESENT", true, in.has_sync ? 1.0 : 0.0, 0.0, "not required");
        add_check(rep.checks, "GATE.SYNC.MARGIN_MIN", true, in.has_sync ? in.sync.metrics.margin : 0.0, thr.sync_margin_min, "not required");
    }

    // Structures
    if (thr.require_struct_ok) {
        if (!in.has_struct) {
            add_check(rep.checks, "GATE.STRUCT.PRESENT", false, 0.0, 1.0, "structures required but not evaluated");
        } else {
            add_check(rep.checks, "GATE.STRUCT.REPORT_OK", in.struct_out.report.ok(), in.struct_out.report.ok() ? 1.0 : 0.0, 1.0,
                      in.struct_out.report.ok() ? "" : "struct/gearbox feasibility report contains failing checks");
        }
    } else {
        add_check(rep.checks, "GATE.STRUCT.PRESENT", true, in.has_struct ? 1.0 : 0.0, 0.0, "not required");
    }

    // Mission
    if (in.has_mission) {
        gate_leq(rep.checks, "GATE.MISSION.SCORE_MAX", in.mission.score, thr.mission_score_max, "mission score exceeds max");
        gate_leq(rep.checks, "GATE.MISSION.TIME_MAX_S", in.mission.total_time_s, thr.mission_time_max_s, "mission time exceeds max");
    } else {
        add_check(rep.checks, "GATE.MISSION.SCORE_MAX", true, 0.0, thr.mission_score_max, "not evaluated");
        add_check(rep.checks, "GATE.MISSION.TIME_MAX_S", true, 0.0, thr.mission_time_max_s, "not evaluated");
    }

    // Compliance
    if (thr.require_compliance_ok) {
        if (!in.has_compliance) {
            add_check(rep.checks, "GATE.COMPLIANCE.PRESENT", false, 0.0, 1.0, "compliance required but not evaluated");
        } else {
            add_check(rep.checks, "GATE.COMPLIANCE.OK", in.compliance.ok(), in.compliance.ok() ? 1.0 : 0.0, 1.0,
                      in.compliance.ok() ? "" : "compliance report fails one or more clauses");
        }
    } else {
        add_check(rep.checks, "GATE.COMPLIANCE.PRESENT", true, in.has_compliance ? 1.0 : 0.0, 0.0, "not required");
    }

    // SFCS
    if (thr.require_sfcs_ok) {
        if (!in.has_sfcs) {
            add_check(rep.checks, "GATE.SFCS.PRESENT", false, 0.0, 1.0, "SFCS required but not evaluated");
        } else {
            add_check(rep.checks, "GATE.SFCS.OK", in.sfcs.ok(), in.sfcs.ok() ? 1.0 : 0.0, 1.0,
                      in.sfcs.ok() ? "" : "SFCS corridor report contains failing checks");
        }
    } else {
        add_check(rep.checks, "GATE.SFCS.PRESENT", true, in.has_sfcs ? 1.0 : 0.0, 0.0, "not required");
    }

    // Final verdict
    bool all_pass = true;
    for (const auto& c : rep.checks) {
        if (!c.pass) { all_pass = false; break; }
    }
    rep.verdict = all_pass ? Verdict::Go : Verdict::NoGo;

    return rep;
}

} // namespace lift::closeout

// -----------------------------
// Lightweight GO/NO-GO thresholds (single-rotor BEMT gating)
// -----------------------------
namespace lift::closeout {

struct GoNoGoThresholds final {
    double min_thrust_N = 0.0;
    double max_power_W = 0.0;
    double max_residual = 0.0;
    double min_disk_area_m2 = 0.0;
    double min_FM = 0.0;

    void validate() const {
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_thrust_N) && min_thrust_N >= 0.0,
                          lift::bemt::ErrorCode::InvalidConfig, "min_thrust_N invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_power_W) && max_power_W >= 0.0,
                          lift::bemt::ErrorCode::InvalidConfig, "max_power_W invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max_residual) && max_residual >= 0.0,
                          lift::bemt::ErrorCode::InvalidConfig, "max_residual invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_disk_area_m2) && min_disk_area_m2 >= 0.0,
                          lift::bemt::ErrorCode::InvalidConfig, "min_disk_area_m2 invalid");
        LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min_FM) && min_FM >= 0.0,
                          lift::bemt::ErrorCode::InvalidConfig, "min_FM invalid");
    }
};

} // namespace lift::closeout
