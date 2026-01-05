// ============================================================================
// Fragment 3.3.04 — Probabilistic GO/NO-GO Gates (P(metric ≥ threshold) ≥ p_min)
// File: prob_gates.cpp
// ============================================================================

#include "prob_gates.hpp"

#include <algorithm>
#include <sstream>

namespace lift::bemt::prob {

namespace {

const EmpiricalCdf* find_cdf(const std::vector<std::pair<std::string, EmpiricalCdf>>& cdfs,
                             const std::string& metric) {
    for (const auto& kv : cdfs) {
        if (kv.first == metric) return &kv.second;
    }
    return nullptr;
}

std::string fmt_gate(const ProbGateEval& e) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os.precision(4);

    if (e.kind == ProbGateKind::ExceedGE) {
        os << "P(" << e.metric << " >= " << e.threshold << ")=" << e.p_value
           << " (min " << e.p_min << ")";
    } else {
        os << "P(" << e.metric << " <= " << e.threshold << ")=" << e.p_value
           << " (min " << e.p_min << ")";
    }
    return os.str();
}

} // namespace

ProbGateReport eval_prob_gates(const std::string& case_id,
                               const std::vector<ProbGate>& gates,
                               const std::vector<std::pair<std::string, EmpiricalCdf>>& cdfs) {
    ProbGateReport rep;
    rep.case_id = case_id;

    for (const auto& g : gates) {
        ProbGate gate = g;
        gate.validate();

        ProbGateEval e;
        e.key = gate.key;
        e.metric = gate.metric;
        e.kind = gate.kind;
        e.threshold = gate.threshold;
        e.p_min = gate.p_min;

        const EmpiricalCdf* cdf = find_cdf(cdfs, gate.metric);
        if (!cdf || cdf->empty()) {
            e.p_value = 0.0;
            e.pass = false;
            e.message = "Missing CDF for metric: " + gate.metric;
        } else {
            if (gate.kind == ProbGateKind::ExceedGE) {
                e.p_value = cdf->exceed(gate.threshold);
            } else {
                // P(X <= thr) = CDF(thr)
                e.p_value = cdf->cdf(gate.threshold);
            }
            e.p_value = clamp(e.p_value, 0.0, 1.0);
            e.pass = (e.p_value + 1e-12 >= e.p_min);
            e.message = fmt_gate(e);
        }

        if (!e.pass) {
            rep.pass_all = false;
            rep.fail_keys.push_back(e.key);
            rep.fail_messages.push_back(e.message);
        }

        rep.evals.push_back(std::move(e));
    }

    rep.code = ErrorCode::Ok;
    rep.message = rep.pass_all ? "PASS" : "FAIL";
    return rep;
}

} // namespace lift::bemt::prob
