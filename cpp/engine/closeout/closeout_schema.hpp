// ============================================================================
// Fragment 3.1.18 — Closeout Record Schema (Deterministic Columns + Serialize Helpers) (C++)
// File: closeout_schema.hpp
// ============================================================================
//
// Purpose:
// - Standardize “closeout” row data for optimization runs.
// - Deterministic column order and stable formatting (CSV-ready).
// - Holds: core performance metrics + diagnostics summary + optional stats summary.
//
// Aligns with requirements:
// - Numerical GO/NO-GO thresholds (hook fields for threshold checks later).
// - Stats hooks: mean/std/min/max (from OnlineStats).
// - Diagnostics: flags, error code, summary.
//
// This file does NOT write to disk. It defines schema and string formatting only.
// Logging/writing is handled in a later fragment.
//
// ============================================================================

#pragma once
#include "../physics/bemt_diagnostics.hpp"
#include "../stats/online_stats.hpp"
#include "../physics/bemt_safety.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

namespace lift::closeout {

struct CloseoutRow final {
    // --- identifiers ---
    std::uint64_t run_id = 0;
    std::uint64_t design_id = 0;
    std::uint64_t eval_id = 0;

    // --- primary outputs ---
    double mass_kg = 0.0;
    double payload_kg = 0.0;

    double T_N = 0.0;
    double P_W = 0.0;
    double Q_Nm = 0.0;

    double FM = 0.0;
    double eta = 0.0;

    double disk_area_m2 = 0.0;
    double disk_loading_N_m2 = 0.0;

    // --- derived mission scoring placeholders ---
    double time_s = 0.0;
    double energy_Wh = 0.0;
    double score = 0.0;

    // --- diagnostics ---
    lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
    std::uint32_t diag_flags = 0;
    int iters = 0;
    double torque_power_rel_err = 0.0;
    std::string diag_summary;

    // --- optional uncertainty summaries ---
    std::uint64_t mc_n = 0;
    double mc_T_mean = 0.0, mc_T_std = 0.0, mc_T_min = 0.0, mc_T_max = 0.0;
    double mc_P_mean = 0.0, mc_P_std = 0.0, mc_P_min = 0.0, mc_P_max = 0.0;
    double mc_FM_mean = 0.0, mc_FM_std = 0.0, mc_FM_min = 0.0, mc_FM_max = 0.0;
};

// Deterministic column list (stable)
inline std::vector<std::string> closeout_columns() {
    return {
        "run_id","design_id","eval_id",
        "mass_kg","payload_kg",
        "T_N","P_W","Q_Nm",
        "FM","eta",
        "disk_area_m2","disk_loading_N_m2",
        "time_s","energy_Wh","score",
        "code","diag_flags","iters","torque_power_rel_err","diag_summary",
        "mc_n",
        "mc_T_mean","mc_T_std","mc_T_min","mc_T_max",
        "mc_P_mean","mc_P_std","mc_P_min","mc_P_max",
        "mc_FM_mean","mc_FM_std","mc_FM_min","mc_FM_max"
    };
}

// Internal: fixed formatting for floats to keep CSV stable across locales.
inline std::string fmt_f(double x, int prec = 6) {
    if (!lift::bemt::is_finite(x)) x = 0.0;
    std::ostringstream ss;
    ss.setf(std::ios::fixed);
    ss << std::setprecision(prec) << x;
    return ss.str();
}

// Escape for CSV: quote if contains comma/quote/newline.
inline std::string csv_escape(const std::string& s) {
    bool need = false;
    for (char c : s) {
        if (c == ',' || c == '"' || c == '\n' || c == '\r') { need = true; break; }
    }
    if (!need) return s;

    std::string out;
    out.reserve(s.size() + 2);
    out.push_back('"');
    for (char c : s) {
        if (c == '"') out.push_back('"');
        out.push_back(c);
    }
    out.push_back('"');
    return out;
}

// Serialize one row to a CSV line (matching closeout_columns order).
inline std::string to_csv_line(const CloseoutRow& r) {
    std::ostringstream ss;

    auto put = [&](const std::string& v) {
        if (ss.tellp() > 0) ss << ",";
        ss << v;
    };

    put(std::to_string(r.run_id));
    put(std::to_string(r.design_id));
    put(std::to_string(r.eval_id));

    put(fmt_f(r.mass_kg));
    put(fmt_f(r.payload_kg));

    put(fmt_f(r.T_N));
    put(fmt_f(r.P_W));
    put(fmt_f(r.Q_Nm));

    put(fmt_f(r.FM));
    put(fmt_f(r.eta));

    put(fmt_f(r.disk_area_m2));
    put(fmt_f(r.disk_loading_N_m2));

    put(fmt_f(r.time_s));
    put(fmt_f(r.energy_Wh));
    put(fmt_f(r.score));

    put(std::to_string(static_cast<unsigned>(r.code)));
    put(std::to_string(r.diag_flags));
    put(std::to_string(r.iters));
    put(fmt_f(r.torque_power_rel_err));
    put(csv_escape(r.diag_summary));

    put(std::to_string(r.mc_n));

    put(fmt_f(r.mc_T_mean)); put(fmt_f(r.mc_T_std)); put(fmt_f(r.mc_T_min)); put(fmt_f(r.mc_T_max));
    put(fmt_f(r.mc_P_mean)); put(fmt_f(r.mc_P_std)); put(fmt_f(r.mc_P_min)); put(fmt_f(r.mc_P_max));
    put(fmt_f(r.mc_FM_mean)); put(fmt_f(r.mc_FM_std)); put(fmt_f(r.mc_FM_min)); put(fmt_f(r.mc_FM_max));

    return ss.str();
}

// Attach MC summaries from OnlineStats
inline void attach_mc_summary(CloseoutRow& r,
                              const lift::stats::OnlineStats& T,
                              const lift::stats::OnlineStats& P,
                              const lift::stats::OnlineStats& FM) {
    r.mc_n = T.count();

    auto tS = lift::stats::summarize(T);
    auto pS = lift::stats::summarize(P);
    auto fS = lift::stats::summarize(FM);

    r.mc_T_mean = tS.mean; r.mc_T_std = tS.std_sample; r.mc_T_min = tS.min_v; r.mc_T_max = tS.max_v;
    r.mc_P_mean = pS.mean; r.mc_P_std = pS.std_sample; r.mc_P_min = pS.min_v; r.mc_P_max = pS.max_v;
    r.mc_FM_mean = fS.mean; r.mc_FM_std = fS.std_sample; r.mc_FM_min = fS.min_v; r.mc_FM_max = fS.max_v;
}

} // namespace lift::closeout
