#include "engine/analysis/closeout_eval2.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_set>

#include "engine/analysis/closeout_error.hpp"

namespace lift {
namespace {

inline bool is_set(double v) {
  return std::isfinite(v);
}

inline double nan_unset() {
  return std::numeric_limits<double>::quiet_NaN();
}

// Deterministic issue de-dupe key (kind+code+context).
struct IssueKey {
  IssueKind kind;
  std::string code;
  std::string context;

  bool operator==(const IssueKey& o) const {
    return kind == o.kind && code == o.code && context == o.context;
  }
};

struct IssueKeyHash {
  std::size_t operator()(const IssueKey& k) const noexcept {
    std::hash<std::string> hs;
    std::size_t h = static_cast<std::size_t>(k.kind);
    h ^= (hs(k.code) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
    h ^= (hs(k.context) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2));
    return h;
  }
};

void add_issue_once(CloseoutReport& r,
                    std::unordered_set<IssueKey, IssueKeyHash>& seen,
                    IssueKind kind,
                    std::string code,
                    std::string message,
                    std::string context) {
  IssueKey key{kind, code, context};
  if (seen.find(key) != seen.end()) return;
  seen.insert(key);

  Issue is;
  is.kind = kind;
  is.code = std::move(code);
  is.message = std::move(message);
  is.context = std::move(context);
  r.issues.push_back(std::move(is));
}

// Gate promotion logic: NoGo > NeedsData > Warn > Go
GateStatus merge_gate(GateStatus a, GateStatus b) {
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

GateStatus gate_from_checks(bool any_no_go, bool any_needs_data, bool any_warn) {
  if (any_no_go) return GateStatus::kNoGo;
  if (any_needs_data) return GateStatus::kNeedsData;
  if (any_warn) return GateStatus::kWarn;
  return GateStatus::kGo;
}

}  // namespace

void evaluate_closeout(CloseoutReport& report, const CloseoutEvalConfig& cfg) {
  // Clear gates first (deterministic)
  report.gates.mass_gate = GateStatus::kGo;
  report.gates.disk_area_gate = GateStatus::kGo;
  report.gates.power_gate = GateStatus::kGo;

  // Keep issues stable: do NOT clear automatically if you want incremental evaluation.
  // If you want a clean re-eval, caller should clear report.issues before calling.

  std::unordered_set<IssueKey, IssueKeyHash> seen;
  seen.reserve(report.issues.size() + 32);
  for (const auto& is : report.issues) {
    seen.insert(IssueKey{is.kind, is.code, is.context});
  }

  // ---------------------------------------------------------------------------
  // MASS: recompute delta_mass_total_kg only if we have a breakdown, and preserve
  // NaN-as-unset semantics when inputs are missing.
  // ---------------------------------------------------------------------------
  {
    bool any_no_go = false;
    bool any_needs_data = false;
    bool any_warn = false;

    const bool has_items = !report.mass_items.empty();

    if (cfg.require_mass_breakdown && !has_items) {
      // Critical: DO NOT set delta_mass_total_kg to 0.0 here.
      // Keep it as-is (could be NaN or caller-provided), but gate is NeedsData.
      any_needs_data = true;
      add_issue_once(
          report, seen,
          IssueKind::kNeedsData,
          "MASS_BREAKDOWN_MISSING",
          "Mass breakdown is required but no mass_items were provided.",
          "mass_items");
    }

    if (has_items) {
      double sum = 0.0;
      bool any_unset_item = false;

      for (const auto& it : report.mass_items) {
        if (!is_set(it.delta_mass_kg)) {
          any_unset_item = true;
          any_needs_data = true;
          add_issue_once(
              report, seen,
              IssueKind::kNeedsData,
              "MASS_ITEM_UNSET",
              "A mass item has delta_mass_kg unset (NaN/Inf).",
              std::string("mass_items:") + it.name);
          continue;
        }
        sum += it.delta_mass_kg;
      }

      // If any item was unset, total must remain unset to preserve contract.
      if (any_unset_item) {
        report.metrics.delta_mass_total_kg = nan_unset();
      } else {
        report.metrics.delta_mass_total_kg = sum;
      }
    }

    // Validate total if set.
    if (is_set(report.metrics.delta_mass_total_kg)) {
      if (cfg.max_delta_mass_total_kg.has_value() &&
          report.metrics.delta_mass_total_kg > *cfg.max_delta_mass_total_kg) {
        any_no_go = true;
        add_issue_once(
            report, seen,
            IssueKind::kError,
            "DELTA_MASS_EXCEEDS_LIMIT",
            "delta_mass_total_kg exceeds configured maximum.",
            "metrics.delta_mass_total_kg");
      }
    } else {
      // If breakdown exists but total is unset, this is NeedsData (or an internal bug)
      // depending on whether we had any unset items.
      if (has_items) {
        // already flagged per-item as NeedsData; keep it consistent
        any_needs_data = true;
      } else if (!cfg.require_mass_breakdown) {
        // If breakdown not required and total is unset, still NeedsData for mass gate.
        any_needs_data = true;
        add_issue_once(
            report, seen,
            IssueKind::kNeedsData,
            "DELTA_MASS_TOTAL_UNSET",
            "delta_mass_total_kg is unset (NaN/Inf).",
            "metrics.delta_mass_total_kg");
      }
    }

    report.gates.mass_gate = gate_from_checks(any_no_go, any_needs_data, any_warn);
  }

  // ---------------------------------------------------------------------------
  // DISK AREA: presence + basic validity + optional threshold.
  // ---------------------------------------------------------------------------
  {
    bool any_no_go = false;
    bool any_needs_data = false;
    bool any_warn = false;

    const double a = report.metrics.disk_area_m2;

    if (!is_set(a)) {
      any_needs_data = true;
      add_issue_once(
          report, seen,
          IssueKind::kNeedsData,
          "DISK_AREA_UNSET",
          "disk_area_m2 is unset (NaN/Inf).",
          "metrics.disk_area_m2");
    } else if (a <= 0.0) {
      any_no_go = true;
      add_issue_once(
          report, seen,
          IssueKind::kError,
          "DISK_AREA_NONPOSITIVE",
          "disk_area_m2 must be > 0.",
          "metrics.disk_area_m2");
    } else if (cfg.min_disk_area_m2.has_value() && a < *cfg.min_disk_area_m2) {
      any_no_go = true;
      add_issue_once(
          report, seen,
          IssueKind::kError,
          "DISK_AREA_BELOW_MIN",
          "disk_area_m2 is below configured minimum.",
          "metrics.disk_area_m2");
    }

    report.gates.disk_area_gate = gate_from_checks(any_no_go, any_needs_data, any_warn);
  }

  // ---------------------------------------------------------------------------
  // POWER (hover): presence + basic validity + optional threshold.
  // ---------------------------------------------------------------------------
  {
    bool any_no_go = false;
    bool any_needs_data = false;
    bool any_warn = false;

    const double p = report.metrics.power_hover_kw;

    if (!is_set(p)) {
      any_needs_data = true;
      add_issue_once(
          report, seen,
          IssueKind::kNeedsData,
          "POWER_HOVER_UNSET",
          "power_hover_kw is unset (NaN/Inf).",
          "metrics.power_hover_kw");
    } else if (p <= 0.0) {
      any_no_go = true;
      add_issue_once(
          report, seen,
          IssueKind::kError,
          "POWER_HOVER_NONPOSITIVE",
          "power_hover_kw must be > 0.",
          "metrics.power_hover_kw");
    } else if (cfg.max_power_hover_kw.has_value() && p > *cfg.max_power_hover_kw) {
      any_no_go = true;
      add_issue_once(
          report, seen,
          IssueKind::kError,
          "POWER_HOVER_EXCEEDS_MAX",
          "power_hover_kw exceeds configured maximum.",
          "metrics.power_hover_kw");
    }

    report.gates.power_gate = gate_from_checks(any_no_go, any_needs_data, any_warn);
  }

  // Optional: If any gate is NeedsData/NoGo, add a summary issue once (deterministic).
  {
    const GateStatus g =
        merge_gate(report.gates.mass_gate,
        merge_gate(report.gates.disk_area_gate, report.gates.power_gate));

    if (g == GateStatus::kNeedsData) {
      add_issue_once(
          report, seen,
          IssueKind::kNeedsData,
          "CLOSEOUT_NEEDS_DATA",
          "Closeout evaluation requires additional inputs to determine Go/NoGo.",
          "closeout.summary");
    } else if (g == GateStatus::kNoGo) {
      add_issue_once(
          report, seen,
          IssueKind::kError,
          "CLOSEOUT_NO_GO",
          "Closeout evaluation indicates No-Go based on one or more gates.",
          "closeout.summary");
    }
  }
}

}  // namespace lift

