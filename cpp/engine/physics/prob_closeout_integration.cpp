// ============================================================================
// Fragment 3.3.09 — Probability Closeout Integration (Select Top-N → Run MC → Emit CSVs)
// File: prob_closeout_integration.cpp
// ============================================================================

#include "prob_closeout_integration.hpp"

#include <algorithm>
#include <limits>

namespace lift::bemt::prob {

namespace {

struct Candidate final {
    std::string case_id;
    double sort_key = 0.0; // smaller is better for LowestHoverPower; larger for HighestFM
};

double fallback_or(double v, double fallback) noexcept {
    return is_finite(v) ? v : fallback;
}

std::uint64_t mix_seed(std::uint64_t base, const std::string& s) {
    // Simple FNV-1a over string seeded with base.
    std::uint64_t h = base ? base : 1469598103934665603ull;
    for (unsigned char c : s) {
        h ^= c;
        h *= 1099511628211ull;
    }
    return h;
}

} // namespace

ProbCloseoutOutputs run_probability_closeout(const std::vector<CloseoutRow>& closeout_rows,
                                             const std::vector<GoNoGoReport>& gonogo_reports,
                                             const std::unordered_map<std::string, McInputs>& baseline_inputs,
                                             const McRunner& runner,
                                             const ProbCloseoutConfig& cfg_in,
                                             const std::unordered_map<std::string, ProbCaseOverrides>& overrides) {
    ProbCloseoutConfig cfg = cfg_in;
    cfg.validate();

    ProbCloseoutOutputs out;

    // Build lookup for GO/NO-GO status
    std::unordered_map<std::string, GoNoGoStatus> status;
    status.reserve(gonogo_reports.size());
    for (const auto& g : gonogo_reports) status[g.case_id] = g.status;

    // Collect candidates
    std::vector<Candidate> cands;
    cands.reserve(closeout_rows.size());

    for (const auto& r : closeout_rows) {
        const bool is_go = status[r.case_id] == GoNoGoStatus::Go;
        if (cfg.promote.require_go && !is_go) continue;

        Candidate c;
        c.case_id = r.case_id;
        if (cfg.promote.sort == ProbPromoteSort::LowestHoverPower) {
            c.sort_key = fallback_or(r.hover_P_W, std::numeric_limits<double>::infinity());
        } else {
            c.sort_key = -fallback_or(r.hover_FM, -std::numeric_limits<double>::infinity());
        }
        cands.push_back(c);
    }

    out.n_candidates = cands.size();
    if (cands.empty()) {
        out.code = ErrorCode::InvalidInput;
        out.message = "No candidates for probability closeout";
        return out;
    }

    // Sort and trim
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        return a.sort_key < b.sort_key;
    });
    if (cands.size() > cfg.promote.top_n) cands.resize(cfg.promote.top_n);
    out.n_promoted = cands.size();

    // Run MC for each promoted case
    out.mc_results.clear();
    out.mc_results.reserve(out.n_promoted);

    for (const auto& c : cands) {
        auto base_it = baseline_inputs.find(c.case_id);
        LIFT_BEMT_REQUIRE(base_it != baseline_inputs.end(), ErrorCode::InvalidInput, "Missing baseline McInputs");
        McInputs base = base_it->second;

        McConfig mc_cfg = cfg.mc;

        auto ov = overrides.find(c.case_id);
        if (ov != overrides.end()) {
            if (ov->second.required_thrust_N >= 0.0) mc_cfg.required_thrust_N = ov->second.required_thrust_N;
            if (ov->second.hover_power_cap_W >= 0.0) mc_cfg.hover_power_cap_W = ov->second.hover_power_cap_W;
        }

        // Derive deterministic seed per case
        mc_cfg.sampler.seed = mix_seed(cfg.seed_base, c.case_id);

        auto res = run_bemt_monte_carlo(c.case_id, base, runner, mc_cfg, cfg.gates);
        out.mc_results.push_back(std::move(res));
    }

    // Flatten summaries
    std::vector<ProbSummary> all_summaries;
    std::vector<ProbGateReport> gate_reports;
    for (const auto& r : out.mc_results) {
        all_summaries.insert(all_summaries.end(), r.summaries.begin(), r.summaries.end());
        if (!r.prob_gate_csv.empty()) {
            gate_reports.push_back(r.gate_report);
        }
    }

    out.prob_closeout_csv = prob_csv(all_summaries);
    if (!gate_reports.empty()) {
        out.prob_gates_csv = prob_gate_csv(gate_reports);
    }

    out.code = ErrorCode::Ok;
    out.message = "OK";
    return out;
}

} // namespace lift::bemt::prob
