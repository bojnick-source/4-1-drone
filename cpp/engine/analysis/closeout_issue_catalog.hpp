#pragma once

#include <string>
#include <string_view>

#include "engine/analysis/closeout_types.hpp"

namespace lift {

/// Central catalog for issue "code" strings.
/// Goal: prevent drift (typos / inconsistent codes) and enforce stable semantics.
///
/// Rules enforced by validator:
/// - code must be non-empty and one of the catalog codes (unless allow_unknown_codes=true)
/// - message must be non-empty
/// - context must be non-empty for non-info issues (configurable)
/// - kind must match the code's declared severity class (optional strictness)
struct IssueCatalogOptions {
  bool allow_unknown_codes = false;
  bool require_context_for_non_info = true;
  bool strict_kind_matching = true;

  // guardrails: avoid megabyte strings in JSON
  size_t max_code_len = 64;
  size_t max_message_len = 512;
  size_t max_context_len = 256;
};

/// Returns true if the code is recognized by the catalog.
bool is_known_issue_code(std::string_view code);

/// Returns the "expected" IssueKind for a code (only valid if is_known_issue_code(code)==true).
IssueKind expected_kind_for_code(std::string_view code);

/// Validate (and optionally normalize) a CloseoutReport in-place:
/// - Adds IssueKind::kError issues for violations (deterministic order)
/// - Does NOT remove existing issues
/// - Does NOT mutate numeric metrics (only validates issues + basic gate consistency)
void validate_closeout_report(CloseoutReport& report, const IssueCatalogOptions& opt = {});

}  // namespace lift

