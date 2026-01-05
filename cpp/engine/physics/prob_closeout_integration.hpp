// ============================================================================
// Fragment 3.3.09 — Probability Closeout Integration (Select Top-N → Run MC → Emit CSVs)
// File: prob_closeout_integration.hpp
// ============================================================================
//
// Purpose:
// - Glue layer: take BEMT closeout rows + GO/NO-GO reports and run Monte Carlo
//   uncertainty only for a selected subset (Top-N), producing:
//    1) prob_closeout.csv  (metric distribution summaries)
//    2) prob_gates.csv     (probabilistic pass/fail)
// - Keeps MC heavy-ish work out of the main optimizer loop.
// - Deterministic seeding for reproducibility.
//
// Inputs:
// - closeout_rows: raw BEMT closeout outputs per case_id
// - gonogo: deterministic gates
// - baseline McInputs per case_id
// - runner: caller-provided BEMT runner (McInputs -> McOutputs)
//
// Selection policy:
// - promote only GO cases if require_go=true
// - rank by lowest hover power (or FM, etc.) using closeout fields
//
// ============================================================================

#pragma once
#include "bemt_closeout_csv.hpp"
#include "closeout_thresholds.hpp"
#include "cdf_report_csv.hpp"
#include "prob_closeout_csv.hpp"
#include "bemt_mc.hpp"

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lift::bemt::prob {

enum class ProbPromoteSort : std::uint8_t {
    LowestHoverPower = 0,
    HighestFM = 1
};

struct ProbPromotePolicy final {
    std::size_t top_n = 25;
    bool require_go = true;
    ProbPromoteSort sort = ProbPromoteSort::LowestHoverPower;

    void validate() const {
        LIFT_BEMT_REQUIRE(top_n >= 1 && top_n <= 1000000, ErrorCode::InvalidConfig, "ProbPromotePolicy.top_n invalid");
    }
};

struct ProbCloseoutConfig final {
    ProbPromotePolicy promote;

    // Monte Carlo config template (per-case values like required_thrust can override)
    McConfig mc;

    // Base seed; per case_id a deterministic derived seed is used.
    std::uint64_t seed_base = 12345;

    // Probabilistic gates to apply (optional)
    std::vector<ProbGate> gates;

    void validate() const {
        promote.validate();
        mc.validate();
        for (const auto& g : gates) g.validate();
    }
};

struct ProbCloseoutOutputs final {
    // Per-case results (only promoted cases)
    std::vector<McResult> mc_results;

    // Flattened CSVs
    std::string prob_closeout_csv; // uses prob_csv() summaries (one row per metric per case)
    std::string prob_gates_csv;    // one row per case gate report

    // Accounting
    std::size_t n_candidates = 0;
    std::size_t n_promoted = 0;

    ErrorCode code = ErrorCode::Ok;
    std::string message;
};

// Per-case overrides for required thrust/power caps.
// If absent, uses cfg.mc required_thrust_N / hover_power_cap_W.
struct ProbCaseOverrides final {
    double required_thrust_N = -1.0;
    double hover_power_cap_W = -1.0;
};

// Main integration entrypoint.
//
// baseline_inputs: case_id -> McInputs baseline
// overrides: optional per-case caps/thrust requirements
ProbCloseoutOutputs run_probability_closeout(const std::vector<CloseoutRow>& closeout_rows,
                                             const std::vector<GoNoGoReport>& gonogo_reports,
                                             const std::unordered_map<std::string, McInputs>& baseline_inputs,
                                             const McRunner& runner,
                                             const ProbCloseoutConfig& cfg,
                                             const std::unordered_map<std::string, ProbCaseOverrides>& overrides = {});

} // namespace lift::bemt::prob
