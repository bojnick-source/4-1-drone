#include "engine/analysis/closeout_issue_catalog.hpp"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lift {
namespace {

struct CodeInfo {
  IssueKind expected_kind;
};

const std::unordered_map<std::string_view, CodeInfo>& catalog() {
  // NOTE: string_view keys point to string literals, lifetime is static.
  static const std::unordered_map<std::string_view, CodeInfo> k = {
      // ---- Mass / breakdown ----
      {"MASS_BREAKDOWN_MISSING", {IssueKind::kNeedsData}},
      {"MASS_ITEM_UNSET", {IssueKind::kNeedsData}},
      {"DELTA_MASS_TOTAL_UNSET", {IssueKind::kNeedsData}},
      {"DELTA_MASS_EXCEEDS_LIMIT", {IssueKind::kError}},

      // ---- Disk area ----
      {"DISK_AREA_UNSET", {IssueKind::kNeedsData}},
      {"DISK_AREA_NONPOSITIVE", {IssueKind::kError}},
      {"DISK_AREA_BELOW_MIN", {IssueKind::kError}},

      // ---- Power ----
      {"POWER_HOVER_UNSET", {IssueKind::kNeedsData}},
      {"POWER_HOVER_NONPOSITIVE", {IssueKind::kError}},
      {"POWER_HOVER_EXCEEDS_MAX", {IssueKind::kError}},

      // ---- Summary ----
      {"CLOSEOUT_NEEDS_DATA", {IssueKind::kNeedsData}},
      {"CLOSEOUT_NO_GO", {IssueKind::kError}},

      // ---- JSON / IO / internal ----
      {"JSON_SERIALIZATION_ERROR", {IssueKind::kError}},
      {"JSON_PARSE_ERROR", {IssueKind::kError}},
      {"INVARIANT_VIOLATION", {IssueKind::kError}},
  };
  return k;
}

void add_validator_issue(CloseoutReport& r,
                         std::unordered_set<std::string>& seen,
                         std::string code,
                         std::string message,
                         std::string context) {
  // Deterministic: unique by "code|context" so we don't spam.
  std::string key = code;
  key.push_back('|');
  key += context;

  if (seen.find(key) != seen.end()) return;
  seen.insert(std::move(key));

  Issue is{};
  is.kind = IssueKind::kError;
  is.code = std::move(code);
  is.message = std::move(message);
  is.context = std::move(context);
  r.issues.push_back(std::move(is));
}

void validate_gate_consistency(CloseoutReport& r,
                               std::unordered_set<std::string>& seen) {
  bool has_no_go_summary = false;
  for (const auto& is : r.issues) {
    if (is.kind == IssueKind::kError && is.code == "CLOSEOUT_NO_GO") {
      has_no_go_summary = true;
      break;
    }
  }
  const bool all_go =
      r.gates.mass_gate == GateStatus::kGo &&
      r.gates.disk_area_gate == GateStatus::kGo &&
      r.gates.power_gate == GateStatus::kGo;

  if (has_no_go_summary && all_go) {
    add_validator_issue(
        r, seen,
        "INVARIANT_VIOLATION",
        "Found CLOSEOUT_NO_GO issue while all gates are Go. Gate/issue inconsistency.",
        "gates");
  }
}

}  // namespace

bool is_known_issue_code(std::string_view code) {
  const auto& c = catalog();
  return c.find(code) != c.end();
}

IssueKind expected_kind_for_code(std::string_view code) {
  const auto& c = catalog();
  auto it = c.find(code);
  return it->second.expected_kind;
}

void validate_closeout_report(CloseoutReport& report, const IssueCatalogOptions& opt) {
  std::unordered_set<std::string> seen;
  seen.reserve(64);

  for (size_t i = 0; i < report.issues.size(); ++i) {
    const Issue& is = report.issues[i];

    if (is.code.empty()) {
      add_validator_issue(
          report, seen,
          "INVARIANT_VIOLATION",
          "Issue has empty code.",
          "issues[" + std::to_string(i) + "].code");
      continue;
    }
    if (is.code.size() > opt.max_code_len) {
      add_validator_issue(
          report, seen,
          "INVARIANT_VIOLATION",
          "Issue code length exceeds max_code_len.",
          "issues[" + std::to_string(i) + "].code");
    }

    if (is.message.empty()) {
      add_validator_issue(
          report, seen,
          "INVARIANT_VIOLATION",
          "Issue has empty message.",
          "issues[" + std::to_string(i) + "].message");
    } else if (is.message.size() > opt.max_message_len) {
      add_validator_issue(
          report, seen,
          "INVARIANT_VIOLATION",
          "Issue message length exceeds max_message_len.",
          "issues[" + std::to_string(i) + "].message");
    }

    if (is.context.size() > opt.max_context_len) {
      add_validator_issue(
          report, seen,
          "INVARIANT_VIOLATION",
          "Issue context length exceeds max_context_len.",
          "issues[" + std::to_string(i) + "].context");
    }
    if (opt.require_context_for_non_info && is.kind != IssueKind::kInfo) {
      if (is.context.empty()) {
        add_validator_issue(
            report, seen,
            "INVARIANT_VIOLATION",
            "Non-info issue must include non-empty context.",
            "issues[" + std::to_string(i) + "].context");
      }
    }

    if (!is_known_issue_code(is.code)) {
      if (!opt.allow_unknown_codes) {
        add_validator_issue(
            report, seen,
            "INVARIANT_VIOLATION",
            "Unknown issue code (not in catalog).",
            "issues[" + std::to_string(i) + "].code=" + is.code);
      }
      continue;
    }

    if (opt.strict_kind_matching) {
      const IssueKind exp = expected_kind_for_code(is.code);
      if (is.kind != exp) {
        add_validator_issue(
            report, seen,
            "INVARIANT_VIOLATION",
            "IssueKind does not match catalog expected kind for code.",
            "issues[" + std::to_string(i) + "].code=" + is.code);
      }
    }

    for (char ch : is.code) {
      const bool ok =
          (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') ||
          ch == '_' ;
      if (!ok) {
        add_validator_issue(
            report, seen,
            "INVARIANT_VIOLATION",
            "Issue code contains illegal character (allowed: A-Z 0-9 _).",
            "issues[" + std::to_string(i) + "].code=" + is.code);
        break;
      }
    }
  }

  validate_gate_consistency(report, seen);
}

}  // namespace lift
