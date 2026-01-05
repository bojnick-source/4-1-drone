/*
===============================================================================
Fragment 3.1.20 — GO/NO-GO CSV Layer (Reasons Flattened) (C++)
File: closeout_report_csv.cpp
===============================================================================
*/

#include "closeout_report_csv.hpp"

#include <sstream>
#include <iomanip>

namespace lift::bemt {

namespace {
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

static std::string join_pipe(const std::vector<std::string>& xs) {
    std::string out;
    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (i) out.push_back('|');
        out.append(xs[i]);
    }
    return out;
}
} // namespace

std::string gonogo_csv_header() {
    return "case_id,status,reasons_count,reasons_keys,reasons_messages\n";
}

std::string gonogo_csv_row(const GoNoGoReport& r) {
    std::vector<std::string> keys;
    std::vector<std::string> msgs;
    keys.reserve(r.reasons.size());
    msgs.reserve(r.reasons.size());

    for (const auto& rr : r.reasons) {
        keys.push_back(rr.key);
        msgs.push_back(rr.message);
    }

    const std::string k = join_pipe(keys);
    const std::string m = join_pipe(msgs);

    std::ostringstream os;
    os
      << esc_csv(r.case_id) << ","
      << (r.status == GoNoGoStatus::Go ? "GO" : "NO_GO") << ","
      << r.reasons.size() << ","
      << esc_csv(k) << ","
      << esc_csv(m)
      << "\n";

    return os.str();
}

std::string gonogo_csv(const std::vector<GoNoGoReport>& rs) {
    std::string out;
    out.reserve(128 + rs.size() * 512);
    out.append(gonogo_csv_header());
    for (const auto& r : rs) out.append(gonogo_csv_row(r));
    return out;
}

std::vector<GoNoGoReport> evaluate_all(const std::vector<CloseoutRow>& rows,
                                       const GoNoGoEvaluator& eval,
                                       double A_total_m2_override,
                                       double delta_mass_kg) {
    std::vector<GoNoGoReport> out;
    out.reserve(rows.size());
    for (const auto& r : rows) {
        out.push_back(eval.evaluate(r, A_total_m2_override, delta_mass_kg));
    }
    return out;
}

} // namespace lift::bemt
