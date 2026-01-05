/*
  Fragment 2.7 — Closeout CLI Runner

  Objective
  ---------
  Provide a hardened command-line tool that:
    1) Parses CloseoutReport JSON (null -> NaN)
    2) Evaluates gates (preserves NaN-as-unset, never defaults missing values to 0.0)
    3) Validates issues against catalog (detects code drift / missing context / illegal chars)
    4) Emits CloseoutReport JSON deterministically
    5) Returns deterministic exit codes for CI gating

  Exit codes (deterministic, matches closeout_cli contract)
  ---------------------------------------------------------
    0  => all gates Go
    2  => at least one gate NoGo
    3  => at least one gate NeedsData (or Warn), none NoGo
    1  => tool error (invalid args / parse error / io error)

  Usage
  -----
  closeout_cli --in <path> --out <path> [options]

  Options
  -------
  --in <path>            Input JSON file path. Use "-" for stdin.
  --out <path>           Output JSON file path. Use "-" for stdout.
  --pretty 0|1           Pretty JSON output (default 1).
  --emit-null 0|1        Emit null for unset numeric values (default 1).
  --require-mass-breakdown 0|1   (default 1).

  Thresholds (optional)
  ---------------------
  --max-delta-mass <kg>
  --min-disk-area <m2>
  --max-power-hover <kW>
*/

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include "engine/analysis/closeout_eval2.hpp"
#include "engine/analysis/closeout_issue_catalog.hpp"
#include "engine/analysis/closeout_json3.hpp"
#include "engine/analysis/closeout_json_parse.hpp"
#include "engine/analysis/closeout_types.hpp"

namespace lift {
namespace {

// Exit codes (matches documented closeout_cli contract)
enum class ExitCode : int {
  kGo = 0,
  kError = 1,
  kNoGo = 2,
  kNeedsData = 3,
};

static constexpr int kExitErrorInt = static_cast<int>(ExitCode::kError);

struct Args {
  std::string in_path;
  std::string out_path;

  bool pretty = true;
  bool emit_null = true;
  bool require_mass_breakdown = true;

  std::optional<double> max_delta_mass_kg;
  std::optional<double> min_disk_area_m2;
  std::optional<double> max_power_hover_kw;
};

static void print_usage(std::ostream& os) {
  os <<
    "closeout_cli --in <path|-> --out <path|-> [options]\n"
    "\n"
    "Options:\n"
    "  --pretty 0|1\n"
    "  --emit-null 0|1\n"
    "  --require-mass-breakdown 0|1\n"
    "  --max-delta-mass <kg>\n"
    "  --min-disk-area <m2>\n"
    "  --max-power-hover <kW>\n";
}

static bool parse_bool01(const char* s, bool* out) {
  if (!s || !out) return false;
  if (std::strcmp(s, "1") == 0) { *out = true; return true; }
  if (std::strcmp(s, "0") == 0) { *out = false; return true; }
  return false;
}

static bool parse_double(const char* s, double* out) {
  if (!s || !out) return false;
  char* end = nullptr;
  const double v = std::strtod(s, &end);
  if (end == s || *end != '\0') return false;
  if (!std::isfinite(v)) return false;
  *out = v;
  return true;
}

static bool get_next(int& i, int argc, char** argv, const char** out) {
  if (i + 1 >= argc) return false;
  *out = argv[++i];
  return true;
}

static bool parse_args(int argc, char** argv, Args* a, std::string* err, bool* help_requested) {
  if (!a) return false;

  for (int i = 1; i < argc; ++i) {
    const char* k = argv[i];

    if (std::strcmp(k, "--help") == 0 || std::strcmp(k, "-h") == 0) {
      if (help_requested) *help_requested = true;
      return true;
    }

    if (std::strcmp(k, "--in") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--in requires a value"; return false; }
      a->in_path = v;
      continue;
    }

    if (std::strcmp(k, "--out") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--out requires a value"; return false; }
      a->out_path = v;
      continue;
    }

    if (std::strcmp(k, "--pretty") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--pretty requires 0|1"; return false; }
      if (!parse_bool01(v, &a->pretty)) { if (err) *err = "--pretty must be 0 or 1"; return false; }
      continue;
    }

    if (std::strcmp(k, "--emit-null") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--emit-null requires 0|1"; return false; }
      if (!parse_bool01(v, &a->emit_null)) { if (err) *err = "--emit-null must be 0 or 1"; return false; }
      continue;
    }

    if (std::strcmp(k, "--require-mass-breakdown") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--require-mass-breakdown requires 0|1"; return false; }
      if (!parse_bool01(v, &a->require_mass_breakdown)) { if (err) *err = "--require-mass-breakdown must be 0 or 1"; return false; }
      continue;
    }

    if (std::strcmp(k, "--max-delta-mass") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--max-delta-mass requires a value"; return false; }
      double d = 0.0;
      if (!parse_double(v, &d)) { if (err) *err = "--max-delta-mass must be a finite number"; return false; }
      a->max_delta_mass_kg = d;
      continue;
    }

    if (std::strcmp(k, "--min-disk-area") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--min-disk-area requires a value"; return false; }
      double d = 0.0;
      if (!parse_double(v, &d)) { if (err) *err = "--min-disk-area must be a finite number"; return false; }
      a->min_disk_area_m2 = d;
      continue;
    }

    if (std::strcmp(k, "--max-power-hover") == 0) {
      const char* v = nullptr;
      if (!get_next(i, argc, argv, &v)) { if (err) *err = "--max-power-hover requires a value"; return false; }
      double d = 0.0;
      if (!parse_double(v, &d)) { if (err) *err = "--max-power-hover must be a finite number"; return false; }
      a->max_power_hover_kw = d;
      continue;
    }

    if (err) *err = std::string("Unknown argument: ") + k;
    return false;
  }

  if (a->in_path.empty()) { if (err) *err = "Missing --in"; return false; }
  if (a->out_path.empty()) { if (err) *err = "Missing --out"; return false; }
  return true;
}

static bool read_all_stdin(std::string* out) {
  if (!out) return false;
  std::ostringstream ss;
  ss << std::cin.rdbuf();
  *out = ss.str();
  return true;
}

static bool read_file(const std::string& path, std::string* out) {
  if (!out) return false;
  std::ifstream f(path, std::ios::binary);
  if (!f.good()) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

static bool write_file(const std::string& path, const std::string& data) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f.good()) return false;
  f.write(data.data(), static_cast<std::streamsize>(data.size()));
  return f.good();
}

static GateStatus merge_gate(GateStatus a, GateStatus b) {
  auto rank = [](GateStatus s) -> int {
    switch (s) {
      case GateStatus::kNoGo:      return 4;
      case GateStatus::kNeedsData: return 3;
      case GateStatus::kWarn:      return 2;
      case GateStatus::kGo:        return 1;
      default:                     return 0;
    }
  };
  return (rank(a) >= rank(b)) ? a : b;
}

static GateStatus aggregate_gates(const CloseoutGates& g) {
  return merge_gate(g.mass_gate, merge_gate(g.disk_area_gate, g.power_gate));
}

static int to_exit_code_int(GateStatus s) {
  switch (s) {
    case GateStatus::kGo:        return static_cast<int>(ExitCode::kGo);
    case GateStatus::kNoGo:      return static_cast<int>(ExitCode::kNoGo);
    case GateStatus::kNeedsData: return static_cast<int>(ExitCode::kNeedsData);
    case GateStatus::kWarn:      return static_cast<int>(ExitCode::kNeedsData); // warn treated as NeedsData
  }
  return kExitErrorInt;
}

}  // namespace
}  // namespace lift

int main(int argc, char** argv) {
  using namespace lift;

  Args a{};
  std::string arg_err;
  bool help = false;
  if (!parse_args(argc, argv, &a, &arg_err, &help)) {
    if (!arg_err.empty()) {
      std::cerr << "Argument error: " << arg_err << "\n\n";
    }
    print_usage(std::cerr);
    return kExitErrorInt;
  }
  if (help) {
    print_usage(std::cout);
    return static_cast<int>(ExitCode::kGo);
  }

  // Read input JSON
  std::string input_json;
  if (a.in_path == "-") {
    if (!read_all_stdin(&input_json)) {
      std::cerr << "IO error: failed to read stdin\n";
      return kExitErrorInt;
    }
  } else {
    if (!read_file(a.in_path, &input_json)) {
      std::cerr << "IO error: failed to read file: " << a.in_path << "\n";
      return kExitErrorInt;
    }
  }

  // Parse
  CloseoutReport report{};
  JsonParseError perr{};
  if (!parse_closeout_report_json(std::string_view(input_json), &report, &perr)) {
    std::cerr << "Parse error: " << perr.message << " @ " << perr.line << ":" << perr.col << "\n";
    return kExitErrorInt;
  }

  // Evaluate
  CloseoutEvalConfig cfg{};
  cfg.require_mass_breakdown = a.require_mass_breakdown;
  cfg.max_delta_mass_total_kg = a.max_delta_mass_kg;
  cfg.min_disk_area_m2 = a.min_disk_area_m2;
  cfg.max_power_hover_kw = a.max_power_hover_kw;

  evaluate_closeout(report, cfg);

  // Validate issue catalog / report consistency
  validate_closeout_report(report, IssueCatalogOptions{});

  // Emit output JSON (deterministic)
  JsonWriteOptions jopt{};
  jopt.pretty = a.pretty;
  jopt.emit_null_for_unset = a.emit_null;

  const std::string out_json = closeout_report_to_json(report, jopt);

  // Write output
  if (a.out_path == "-") {
    std::cout << out_json;
    if (jopt.pretty) std::cout << "\n";
    if (!std::cout.good()) {
      std::cerr << "IO error: failed to write stdout\n";
      return kExitErrorInt;
    }
  } else {
    if (!write_file(a.out_path, out_json)) {
      std::cerr << "IO error: failed to write file: " << a.out_path << "\n";
      return kExitErrorInt;
    }
  }

  // Deterministic return code for CI gating
  return to_exit_code_int(aggregate_gates(report.gates));
}
