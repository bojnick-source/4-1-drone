/*
  Fragment 2.5 — Closeout JSON Round-Trip Selftest

  Objective
  ---------
  Provide a hardened, framework-free selftest that validates:
    1) Writer never emits NaN/Inf into JSON (must be null or omitted).
    2) Parser converts JSON null -> internal NaN (unset contract preserved).
    3) Round-trip serialize(parse(serialize(report))) is stable and deterministic.
    4) Gate + Issue enums survive round-trip.

  Why this exists
  ---------------
  GitHub/Codex checks will flag regressions where:
    - Unset numeric fields are incorrectly defaulted to 0.0.
    - JSON output contains NaN (invalid JSON).
    - Parser accepts non-JSON numerics or fails on valid null.
    - Output ordering changes and diffs churn.

  Integration
  ----------
  This file is written to compile standalone and run as a tiny executable test.
  It does NOT require Catch2/GoogleTest/etc.

  Expected use
  ------------
  - Build and run in CI or locally:
      ./closeout_json_selftest
  - Non-zero return code indicates failure.
*/

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>

#include "engine/analysis/closeout_eval2.hpp"
#include "engine/analysis/closeout_json3.hpp"
#include "engine/analysis/closeout_json_parse.hpp"
#include "engine/analysis/closeout_types.hpp"

namespace lift {
namespace {

static int g_fail_count = 0;

void fail(std::string_view msg) {
  ++g_fail_count;
  std::cerr << "[FAIL] " << msg << "\n";
}

void pass(std::string_view msg) {
  std::cerr << "[ OK ] " << msg << "\n";
}

bool is_nan(double v) {
  return std::isnan(v);
}

bool is_finite(double v) {
  return std::isfinite(v);
}

void expect_true(bool v, std::string_view msg) {
  if (!v) fail(msg);
  else pass(msg);
}

void expect_eq_str(const std::string& a, const std::string& b, std::string_view msg) {
  if (a != b) {
    fail(msg);
    std::cerr << "  a: " << a << "\n";
    std::cerr << "  b: " << b << "\n";
  } else {
    pass(msg);
  }
}

void expect_gate(GateStatus got, GateStatus exp, std::string_view msg) {
  if (got != exp) fail(msg);
  else pass(msg);
}

void expect_issue_kind(IssueKind got, IssueKind exp, std::string_view msg) {
  if (got != exp) fail(msg);
  else pass(msg);
}

void expect_nan(double v, std::string_view msg) {
  if (!is_nan(v)) {
    fail(msg);
    std::cerr << "  expected NaN, got " << v << "\n";
  } else {
    pass(msg);
  }
}

void expect_finite(double v, std::string_view msg) {
  if (!is_finite(v)) {
    fail(msg);
    std::cerr << "  expected finite, got NaN/Inf\n";
  } else {
    pass(msg);
  }
}

// Basic check: JSON output must not contain "nan", "NaN", "inf", "Infinity".
void expect_json_has_no_nonfinite_literals(const std::string& json) {
  auto contains = [&](std::string_view needle) -> bool {
    return json.find(std::string(needle)) != std::string::npos;
  };

  expect_true(!contains("nan"), "JSON must not contain 'nan'");
  expect_true(!contains("NaN"), "JSON must not contain 'NaN'");
  expect_true(!contains("inf"), "JSON must not contain 'inf'");
  expect_true(!contains("Inf"), "JSON must not contain 'Inf'");
  expect_true(!contains("Infinity"), "JSON must not contain 'Infinity'");
}

// Build a minimal report that includes:
// - some unset numbers (NaN) to force null emission
// - some set numbers
// - gates / issues / mass items
CloseoutReport make_report_with_mixed_set_unset() {
  CloseoutReport r{};

  // IMPORTANT: NaN-as-unset is the contract.
  r.metrics.delta_mass_total_kg = std::numeric_limits<double>::quiet_NaN();  // unset
  r.metrics.disk_area_m2        = 12.345;                                    // set
  r.metrics.power_hover_kw      = std::numeric_limits<double>::quiet_NaN();  // unset

  // Add a mass breakdown with one unset to ensure evaluator preserves NaN total.
  MassItem a{};
  a.name = "motors";
  a.delta_mass_kg = 1.2;

  MassItem b{};
  b.name = "gearbox";
  b.delta_mass_kg = std::numeric_limits<double>::quiet_NaN(); // unset item

  r.mass_items.push_back(a);
  r.mass_items.push_back(b);

  // Gates + issues get validated through JSON enum string mapping.
  r.gates.mass_gate = GateStatus::kNeedsData;
  r.gates.disk_area_gate = GateStatus::kGo;
  r.gates.power_gate = GateStatus::kNeedsData;

  Issue is{};
  is.kind = IssueKind::kNeedsData;
  is.code = "MASS_ITEM_UNSET";
  is.message = "A mass item is unset.";
  is.context = "mass_items:gearbox";
  r.issues.push_back(is);

  return r;
}

void test_writer_no_nan_literals() {
  CloseoutReport r = make_report_with_mixed_set_unset();

  JsonWriteOptions opt{};
  opt.pretty = true;
  opt.emit_null_for_unset = true;

  const std::string json = closeout_report_to_json(r, opt);

  expect_true(!json.empty(), "Writer produces non-empty JSON");
  expect_json_has_no_nonfinite_literals(json);

  // Should contain null for the unset fields if emit_null_for_unset=true.
  expect_true(json.find("\"delta_mass_total_kg\": null") != std::string::npos ||
              json.find("\"delta_mass_total_kg\":null") != std::string::npos,
              "Unset delta_mass_total_kg must emit null");
}

void test_parser_null_to_nan() {
  // Minimal JSON that exercises null -> NaN conversion.
  const std::string json =
      "{"
      "\"metrics\":{"
        "\"delta_mass_total_kg\":null,"
        "\"disk_area_m2\":1.0,"
        "\"power_hover_kw\":null"
      "},"
      "\"gates\":{"
        "\"mass_gate\":\"NeedsData\","
        "\"disk_area_gate\":\"Go\","
        "\"power_gate\":\"NeedsData\""
      "},"
      "\"mass_items\":["
        "{\"name\":\"m1\",\"delta_mass_kg\":null},"
        "{\"name\":\"m2\",\"delta_mass_kg\":2.0}"
      "],"
      "\"issues\":["
        "{\"kind\":\"NeedsData\",\"code\":\"X\",\"message\":\"Y\",\"context\":\"Z\"}"
      "]"
      "}";

  CloseoutReport out{};
  JsonParseError err{};
  const bool ok = parse_closeout_report_json(std::string_view(json), &out, &err);

  if (!ok) {
    fail("Parser must accept valid JSON with nulls");
    std::cerr << "  parse error: " << err.message << " @ " << err.line << ":" << err.col << "\n";
    return;
  }
  pass("Parser accepted JSON with nulls");

  expect_nan(out.metrics.delta_mass_total_kg, "Parser: null -> NaN for delta_mass_total_kg");
  expect_finite(out.metrics.disk_area_m2, "Parser: disk_area_m2 parsed finite");
  expect_nan(out.metrics.power_hover_kw, "Parser: null -> NaN for power_hover_kw");

  expect_gate(out.gates.mass_gate, GateStatus::kNeedsData, "Parser: mass_gate enum roundtrip");
  expect_gate(out.gates.disk_area_gate, GateStatus::kGo, "Parser: disk_area_gate enum roundtrip");
  expect_gate(out.gates.power_gate, GateStatus::kNeedsData, "Parser: power_gate enum roundtrip");

  expect_true(out.mass_items.size() == 2, "Parser: mass_items size");
  expect_eq_str(out.mass_items[0].name, "m1", "Parser: mass_items[0].name");
  expect_nan(out.mass_items[0].delta_mass_kg, "Parser: mass_items[0].delta_mass_kg null->NaN");
  expect_finite(out.mass_items[1].delta_mass_kg, "Parser: mass_items[1].delta_mass_kg finite");

  expect_true(out.issues.size() == 1, "Parser: issues size");
  expect_issue_kind(out.issues[0].kind, IssueKind::kNeedsData, "Parser: issue kind enum");
}

void test_round_trip_determinism() {
  CloseoutReport r = make_report_with_mixed_set_unset();

  // Run evaluator to ensure it doesn't zero-out totals and produces stable issues.
  CloseoutEvalConfig cfg{};
  cfg.require_mass_breakdown = true;
  evaluate_closeout(r, cfg);

  JsonWriteOptions opt{};
  opt.pretty = false;           // make it maximally deterministic (no whitespace diffs)
  opt.emit_null_for_unset = true;

  const std::string j1 = closeout_report_to_json(r, opt);

  CloseoutReport parsed{};
  JsonParseError err{};
  const bool ok = parse_closeout_report_json(std::string_view(j1), &parsed, &err);
  expect_true(ok, "Round-trip parse of writer output must succeed");

  const std::string j2 = closeout_report_to_json(parsed, opt);

  // Deterministic requirement: output should be stable across round trip.
  // NOTE: This assumes writer key order is stable (it is).
  expect_eq_str(j1, j2, "Round-trip JSON must be bitwise identical (pretty=false)");
}

}  // namespace
}  // namespace lift

int main() {
  using namespace lift;

  test_writer_no_nan_literals();
  test_parser_null_to_nan();
  test_round_trip_determinism();

  if (g_fail_count != 0) {
    std::cerr << "\nSelftest failures: " << g_fail_count << "\n";
    return 1;
  }
  std::cerr << "\nAll selftests passed.\n";
  return 0;
}
