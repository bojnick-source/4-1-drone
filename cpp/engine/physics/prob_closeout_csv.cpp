// ============================================================================
// Fragment 3.3.05 — Probability Closeout CSV (prob_closeout.csv)
// File: prob_closeout_csv.cpp
// ============================================================================

#include "prob_closeout_csv.hpp"

#include <iomanip>
#include <sstream>

namespace lift::bemt::prob {

namespace {

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

} // namespace

std::string prob_gate_csv_header() {
    return
        "case_id,pass_all,code,message,"
        "fail_keys,fail_messages,"
        "eval_count\n";
}

std::string prob_gate_csv_row(const ProbGateReport& r) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(6);

    // Join fails
    std::string keys, msgs;
    for (std::size_t i = 0; i < r.fail_keys.size(); ++i) {
        if (i) keys.push_back('|');
        keys.append(r.fail_keys[i]);
    }
    for (std::size_t i = 0; i < r.fail_messages.size(); ++i) {
        if (i) msgs.push_back('|');
        msgs.append(r.fail_messages[i]);
    }

    os
      << esc_csv(r.case_id) << ","
      << (r.pass_all ? "1" : "0") << ","
      << static_cast<unsigned>(r.code) << ","
      << esc_csv(r.message) << ","
      << esc_csv(keys) << ","
      << esc_csv(msgs) << ","
      << r.evals.size()
      << "\n";

    return os.str();
}

std::string prob_gate_csv(const std::vector<ProbGateReport>& rs) {
    std::string out;
    out.reserve(256 + rs.size() * 240);
    out.append(prob_gate_csv_header());
    for (const auto& r : rs) out.append(prob_gate_csv_row(r));
    return out;
}

} // namespace lift::bemt::prob
