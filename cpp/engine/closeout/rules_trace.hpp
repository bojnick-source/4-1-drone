/*
===============================================================================
Fragment 3.1.34 — Rules Verification Traceability (Clause Mapping + Gate/Metric Evidence Matrix) (C++)
File: cpp/engine/closeout/rules_trace.hpp
===============================================================================
*/

#pragma once

#include "gates.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace lift::closeout {

// -----------------------------
// Clause mapping (primary text)
// -----------------------------
struct ClauseSpec final {
  std::string clause_id;      // stable id used throughout (e.g., "RULE.MASS.EMPTY_MAX")
  std::string title;          // short human title
  std::string source_name;    // e.g., "DARPA LIFT Solicitation"
  std::string source_rev;     // e.g., "Rev A", date, etc.
  std::string section_ref;    // e.g., "Section 3.2.1", "Para (b)(2)", etc.
  std::string excerpt;        // short excerpt (keep <= 25 words when filled)
  std::string interpretation; // your engineering interpretation
  bool mandatory = true;      // mandatory vs guidance
};

// -----------------------------
// Evidence row (traceability matrix)
// -----------------------------
enum class TraceStatus : std::uint8_t { Pass = 0, Fail = 1, Unknown = 2 };

struct TraceRow final {
  std::string clause_id;
  std::string gate_id;
  std::string metric_key;

  TraceStatus status = TraceStatus::Unknown;
  GateSeverity severity = GateSeverity::FailHard;

  double metric_value = 0.0;
  double thr_a = 0.0;
  double thr_b = 0.0;

  std::string message; // failure reason or note
};

struct TraceReport final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;

  std::vector<ClauseSpec> clauses; // optional: included if provided
  std::vector<TraceRow> rows;

  bool hard_pass() const noexcept {
    if (code != lift::bemt::ErrorCode::Ok) return false;
    for (const auto& r : rows) {
      if (r.status == TraceStatus::Fail && r.severity == GateSeverity::FailHard) return false;
    }
    return true;
  }
};

inline TraceStatus to_trace_status(GateStatus s) noexcept {
  switch (s) {
    case GateStatus::Pass: return TraceStatus::Pass;
    case GateStatus::Fail: return TraceStatus::Fail;
    default: return TraceStatus::Unknown;
  }
}

// -----------------------------
// Clause store
// -----------------------------
using ClauseMap = std::unordered_map<std::string, ClauseSpec>;

inline void validate_clause(const ClauseSpec& c) {
  LIFT_BEMT_REQUIRE(!c.clause_id.empty(), lift::bemt::ErrorCode::InvalidConfig, "ClauseSpec.clause_id empty");
}

// -----------------------------
// Trace builder
// -----------------------------
struct TraceBuildConfig final {
  // If true, include clauses that appear in gates but are missing from the clause map
  bool include_missing_clauses = true;

  // If true, inject placeholder clause entries for missing ones
  bool inject_placeholders = true;

  void validate() const {
    // no numeric invariants
  }
};

inline ClauseSpec placeholder_clause(const std::string& clause_id) {
  ClauseSpec c;
  c.clause_id = clause_id;
  c.title = "UNMAPPED CLAUSE (placeholder)";
  c.source_name = "PRIMARY SOURCE REQUIRED";
  c.source_rev = "";
  c.section_ref = "";
  c.excerpt = "";
  c.interpretation = "Populate from primary DARPA text.";
  c.mandatory = true;
  return c;
}

inline TraceReport build_rules_trace(const GateReport& gates_report,
                                     const MetricMap& metrics,
                                     const ClauseMap& clause_map_in,
                                     const TraceBuildConfig& cfg_in = {}) {
  TraceBuildConfig cfg = cfg_in;
  cfg.validate();

  TraceReport tr;

  // copy map so we can inject placeholders deterministically if requested
  ClauseMap clause_map = clause_map_in;
  (void)metrics; // metrics currently unused; placeholder for future evidence enrichment

  // Build rows directly from gate evals (gate->clause->metric evidence)
  tr.rows.reserve(gates_report.evals.size());

  for (const auto& e : gates_report.evals) {
    TraceRow r;
    r.clause_id = e.clause_id.empty() ? e.gate_id : e.clause_id;
    r.gate_id = e.gate_id;
    r.metric_key = e.metric_key;
    r.status = to_trace_status(e.status);
    r.severity = e.severity;
    r.metric_value = e.value;
    r.thr_a = e.a;
    r.thr_b = e.b;
    r.message = e.message;

    tr.rows.push_back(std::move(r));

    // Ensure clause exists if requested
    if (cfg.include_missing_clauses) {
      if (clause_map.find(r.clause_id) == clause_map.end()) {
        if (cfg.inject_placeholders) {
          clause_map.emplace(r.clause_id, placeholder_clause(r.clause_id));
        }
      }
    }
  }

  // Export clause list deterministically: sorted by clause_id
  tr.clauses.reserve(clause_map.size());
  for (const auto& kv : clause_map) {
    validate_clause(kv.second);
    tr.clauses.push_back(kv.second);
  }
  std::sort(tr.clauses.begin(), tr.clauses.end(),
            [](const ClauseSpec& a, const ClauseSpec& b) { return a.clause_id < b.clause_id; });

  tr.code = lift::bemt::ErrorCode::Ok;
  return tr;
}

// -----------------------------
// Deterministic text rendering (for logs / closeout dumps)
// -----------------------------
inline const char* status_str(TraceStatus s) noexcept {
  switch (s) {
    case TraceStatus::Pass: return "PASS";
    case TraceStatus::Fail: return "FAIL";
    default: return "UNKNOWN";
  }
}
inline const char* severity_str(GateSeverity s) noexcept {
  return (s == GateSeverity::FailHard) ? "HARD" : "SOFT";
}

inline std::string render_trace_table_tsv(const TraceReport& tr) {
  // TSV is stable, easy to diff, easy to import into sheets.
  std::ostringstream os;
  os << "clause_id\tgate_id\tmetric_key\tstatus\tseverity\tvalue\tthr_a\tthr_b\tmessage\n";
  for (const auto& r : tr.rows) {
    os << r.clause_id << "\t"
       << r.gate_id << "\t"
       << r.metric_key << "\t"
       << status_str(r.status) << "\t"
       << severity_str(r.severity) << "\t"
       << r.metric_value << "\t"
       << r.thr_a << "\t"
       << r.thr_b << "\t"
       << r.message
       << "\n";
  }
  return os.str();
}

} // namespace lift::closeout
