/*
===============================================================================
Fragment 3.1.63 — Risk Analyzer (ECDF → Pass/Fail Probability + Quantiles + Summary for Closeout) (C++)
File: cpp/engine/stats/risk_analyzer.hpp
===============================================================================
*/

#pragma once

#include "empirical_cdf.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <string>
#include <utility>
#include <vector>

namespace lift::stats {

struct Summary final {
  std::size_t n = 0;
  double min = 0.0;
  double max = 0.0;
  double mean = 0.0;
  double stdev = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(min) || n == 0, lift::bemt::ErrorCode::InvalidInput, "Summary.min invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(max) || n == 0, lift::bemt::ErrorCode::InvalidInput, "Summary.max invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(mean) || n == 0, lift::bemt::ErrorCode::InvalidInput, "Summary.mean invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(stdev) || n == 0, lift::bemt::ErrorCode::InvalidInput, "Summary.stdev invalid");
  }
};

enum class Comparator : std::uint8_t { LE = 0, LT = 1, GE = 2, GT = 3 };

inline const char* comparator_cstr(Comparator c) noexcept {
  switch (c) {
    case Comparator::LE: return "<=";
    case Comparator::LT: return "<";
    case Comparator::GE: return ">=";
    case Comparator::GT: return ">";
    default: return "?";
  }
}

struct ThresholdSpec final {
  std::string metric_id;
  Comparator cmp = Comparator::LE;
  double threshold = 0.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(!metric_id.empty(), lift::bemt::ErrorCode::InvalidConfig, "ThresholdSpec.metric_id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(threshold), lift::bemt::ErrorCode::InvalidConfig, "ThresholdSpec.threshold invalid");
  }
};

struct RiskItem final {
  std::string metric_id;
  std::string comparator;
  double threshold = 0.0;
  double probability = 0.0;
  double fail_probability = 0.0;
  double p50 = 0.0;
  double p90 = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  Summary summary{};

  void validate() const {
    LIFT_BEMT_REQUIRE(!metric_id.empty(), lift::bemt::ErrorCode::InvalidInput, "RiskItem.metric_id empty");
    LIFT_BEMT_REQUIRE(!comparator.empty(), lift::bemt::ErrorCode::InvalidInput, "RiskItem.comparator empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(threshold), lift::bemt::ErrorCode::InvalidInput, "RiskItem.threshold invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(probability) && probability >= 0.0 && probability <= 1.0,
                      lift::bemt::ErrorCode::InvalidInput, "RiskItem.probability invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(fail_probability) && fail_probability >= 0.0 && fail_probability <= 1.0,
                      lift::bemt::ErrorCode::InvalidInput, "RiskItem.fail_probability invalid");
    summary.validate();
  }
};

inline bool ecdf_has_data(const EmpiricalCDF& e) noexcept { return e.size() > 0; }

inline double ecdf_cdf_leq(const EmpiricalCDF& e, double x) noexcept {
  const double p = e.cdf(x);
  if (!lift::bemt::is_finite(p)) return 0.0;
  return lift::bemt::clamp(p, 0.0, 1.0);
}

inline double ecdf_quantile(const EmpiricalCDF& e, double p01) noexcept {
  const double pp = lift::bemt::clamp(p01, 0.0, 1.0);
  const double q = e.quantile(pp);
  return lift::bemt::is_finite(q) ? q : 0.0;
}

inline Summary ecdf_summary(const EmpiricalCDF& e) {
  const auto s = e.summary();
  Summary out;
  out.n = s.n;
  out.min = s.min;
  out.max = s.max;
  out.mean = s.mean;
  out.stdev = s.stdev;
  out.validate();
  return out;
}

inline double pass_probability(const EmpiricalCDF& e, Comparator cmp, double thr) noexcept {
  if (!ecdf_has_data(e) || !lift::bemt::is_finite(thr)) return 0.0;
  const double p_le = ecdf_cdf_leq(e, thr);
  switch (cmp) {
    case Comparator::LE: return p_le;
    case Comparator::LT: return p_le;
    case Comparator::GE: return 1.0 - p_le;
    case Comparator::GT: return 1.0 - p_le;
    default: return 0.0;
  }
}

inline const EmpiricalCDF* find_dist(const std::vector<std::pair<std::string, EmpiricalCDF>>& dists,
                                     const std::string& metric_id) noexcept {
  for (const auto& p : dists) if (p.first == metric_id) return &p.second;
  return nullptr;
}

inline std::vector<RiskItem> build_risk_items(const std::vector<std::pair<std::string, EmpiricalCDF>>& dists,
                                              const std::vector<ThresholdSpec>& thresholds) {
  for (const auto& t : thresholds) t.validate();

  std::vector<RiskItem> out;
  out.reserve(thresholds.size());

  for (const auto& t : thresholds) {
    RiskItem ri;
    ri.metric_id = t.metric_id;
    ri.comparator = comparator_cstr(t.cmp);
    ri.threshold = t.threshold;

    const EmpiricalCDF* e = find_dist(dists, t.metric_id);
    if (!e || !ecdf_has_data(*e)) {
      ri.probability = 0.0;
      ri.fail_probability = 1.0;
      ri.summary = Summary{};
      out.push_back(std::move(ri));
      continue;
    }

    ri.p50 = ecdf_quantile(*e, 0.50);
    ri.p90 = ecdf_quantile(*e, 0.90);
    ri.p95 = ecdf_quantile(*e, 0.95);
    ri.p99 = ecdf_quantile(*e, 0.99);
    ri.summary = ecdf_summary(*e);

    ri.probability = pass_probability(*e, t.cmp, t.threshold);
    ri.fail_probability = 1.0 - ri.probability;
    ri.validate();
    out.push_back(std::move(ri));
  }

  return out;
}

} // namespace lift::stats

