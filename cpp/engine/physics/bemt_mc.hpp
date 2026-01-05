// ============================================================================
// Fragment 3.3.06 — Monte Carlo BEMT Wrapper (Run N samples → CDFs + Prob CSV)
// File: bemt_mc.hpp
// ============================================================================
//
// Purpose:
// - Provide a fast Monte Carlo wrapper around the BEMT solver.
// - Runs N samples by perturbing selected scalar parameters.
// - Collects distributions for key outputs and emits probability summaries.
//
// This module is intentionally "pluggable":
// - You provide a callable that executes BEMT for given inputs and returns outputs.
// - This avoids tight coupling to your exact BEMT interfaces while keeping the
//   probabilistic infrastructure hardened and ready.
//
// Outputs tracked (typical):
// - hover_power_W, hover_FM, hover_thrust_N
// - thrust_margin_N (thrust - required)
// - power_margin_W (P_max - power)
//
// ============================================================================

#pragma once
#include "cdf.hpp"
#include "cdf_report_csv.hpp"
#include "uncertainty.hpp"
#include "prob_gates.hpp"

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lift::bemt::prob {

// Minimal “input knobs” for MC. Extend as needed.
struct McInputs final {
    double rho = 1.225;
    double mu  = 1.81e-5;

    double omega_rad_s = 0.0;
    double collective_rad = 0.0;

    // Geom scale factors (1.0 = baseline)
    double radius_scale = 1.0;
    double chord_scale  = 1.0;
};

// Minimal “outputs” for MC. Extend as needed.
struct McOutputs final {
    ErrorCode code = ErrorCode::Ok;

    double hover_T_N = 0.0;
    double hover_P_W = 0.0;
    double hover_FM  = 0.0;
};

// Caller supplies this: run BEMT for given inputs.
using McRunner = std::function<McOutputs(const McInputs&)>;

struct McConfig final {
    SamplerConfig sampler;

    // Distributions for uncertain inputs; if not provided, inputs stay constant.
    // Use name keys: "rho","mu","omega","collective","radius_scale","chord_scale"
    std::unordered_map<std::string, DistSpec> dists;

    // Required thrust and caps for margin metrics
    double required_thrust_N = 0.0;
    double hover_power_cap_W = 0.0;

    // When true, drop failed solver runs from distributions (recommended)
    bool drop_failed_runs = true;

    void validate() const {
        sampler.validate();
        LIFT_BEMT_REQUIRE(is_finite(required_thrust_N) && required_thrust_N >= 0.0, ErrorCode::InvalidConfig, "required_thrust_N invalid");
        LIFT_BEMT_REQUIRE(is_finite(hover_power_cap_W) && hover_power_cap_W >= 0.0, ErrorCode::InvalidConfig, "hover_power_cap_W invalid");
    }
};

struct McResult final {
    std::string case_id;

    // Raw sample outputs
    std::vector<double> hover_power_W;
    std::vector<double> hover_FM;
    std::vector<double> hover_thrust_N;
    std::vector<double> thrust_margin_N;
    std::vector<double> power_margin_W;

    // Derived CDFs (metric -> cdf)
    std::vector<std::pair<std::string, EmpiricalCdf>> cdfs;

    // Probability summaries (one per metric)
    std::vector<ProbSummary> summaries;

    // Optional probabilistic gate report
    ProbGateReport gate_report;

    // CSV exports
    std::string prob_summary_csv; // from prob_csv()
    std::string prob_gate_csv;    // from prob_gate_csv()

    // Execution stats
    std::size_t n_requested = 0;
    std::size_t n_ok = 0;
    std::size_t n_failed = 0;

    ErrorCode code = ErrorCode::Ok;
    std::string message;
};

// Run Monte Carlo and produce CDF summaries.
McResult run_bemt_monte_carlo(const std::string& case_id,
                              const McInputs& baseline,
                              const McRunner& runner,
                              const McConfig& cfg,
                              const std::vector<ProbGate>& gates);

} // namespace lift::bemt::prob
