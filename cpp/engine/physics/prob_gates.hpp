// ============================================================================
// Fragment 3.3.04 — Probabilistic GO/NO-GO Gates (P(metric ≥ threshold) ≥ p_min)
// File: prob_gates.hpp
// ============================================================================
//
// Purpose:
// - Convert Monte Carlo samples into numeric pass/fail decisions using CDF.
// - Examples:
//    - P(thrust_margin_N >= 0) >= 0.95  (95% chance you meet thrust)
//    - P(hover_power_W <= P_max) >= 0.90  (90% chance you stay under power cap)
//
// Gate types supported:
// - Exceed (>=) gate
// - Not-exceed (<=) gate
//
// Output includes reason strings ready for closeout.
// ============================================================================

#pragma once
#include "cdf.hpp"
#include "bemt_error.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::bemt::prob {

enum class ProbGateKind : std::uint8_t {
    ExceedGE = 0,   // P(X >= thr) >= p_min
    NotExceedLE = 1 // P(X <= thr) >= p_min
};

struct ProbGate final {
    std::string key;        // stable reason key, e.g. "prob_thrust_margin"
    std::string metric;     // name of metric
    ProbGateKind kind = ProbGateKind::ExceedGE;

    double threshold = 0.0;
    double p_min = 0.95;

    void validate() const {
        LIFT_BEMT_REQUIRE(!key.empty(), ErrorCode::InvalidConfig, "ProbGate.key empty");
        LIFT_BEMT_REQUIRE(!metric.empty(), ErrorCode::InvalidConfig, "ProbGate.metric empty");
        LIFT_BEMT_REQUIRE(is_finite(threshold), ErrorCode::InvalidConfig, "ProbGate.threshold not finite");
        LIFT_BEMT_REQUIRE(is_finite(p_min) && p_min > 0.0 && p_min <= 1.0, ErrorCode::InvalidConfig, "ProbGate.p_min invalid");
    }
};

struct ProbGateEval final {
    std::string key;
    std::string metric;

    ProbGateKind kind = ProbGateKind::ExceedGE;
    double threshold = 0.0;

    double p_value = 0.0;      // computed probability (meets condition)
    double p_min = 0.0;

    bool pass = false;

    // Human-readable message for closeout
    std::string message;
};

struct ProbGateReport final {
    std::string case_id;

    ErrorCode code = ErrorCode::Ok;
    std::string message;

    bool pass_all = true;
    std::vector<ProbGateEval> evals;

    // Fail reasons are extracted from evals where pass=false
    std::vector<std::string> fail_keys;
    std::vector<std::string> fail_messages;
};

// Evaluate gates given named empirical CDFs.
// cdfs: metric name -> EmpiricalCdf
ProbGateReport eval_prob_gates(const std::string& case_id,
                               const std::vector<ProbGate>& gates,
                               const std::vector<std::pair<std::string, EmpiricalCdf>>& cdfs);

} // namespace lift::bemt::prob
