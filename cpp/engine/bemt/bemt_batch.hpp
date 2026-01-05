/*
===============================================================================
Fragment 3.1.54 — BEMT Batch Evaluator + Uncertainty Hooks (Metric Capture → ECDF Distributions) (C++)
File: cpp/engine/bemt/bemt_batch.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_core.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include "../stats/empirical_cdf.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace lift::bemt {

enum class SamplePolicy : std::uint8_t { OnlyOk = 0, IncludeAll = 1 };

struct MetricSpec final {
  std::string id;
  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), ErrorCode::InvalidConfig, "MetricSpec.id empty");
  }
};

struct SampleInputs final {
  RotorGeometry geom;
  std::shared_ptr<const IAirfoilDatabase> airfoils;
  Environment env;
  OperatingPoint op;

  void validate() const {
    geom.validate();
    env.validate();
    op.validate();
    LIFT_BEMT_REQUIRE(static_cast<bool>(airfoils), ErrorCode::InvalidInput, "SampleInputs.airfoils missing");
  }
};

struct ISampleProvider {
  virtual ~ISampleProvider() = default;
  virtual bool sample(std::size_t idx, SampleInputs& dst) = 0;
};

struct BatchConfig final {
  std::size_t n_samples = 0;
  SamplePolicy policy = SamplePolicy::OnlyOk;
  std::vector<MetricSpec> metrics;

  void validate() const {
    LIFT_BEMT_REQUIRE(n_samples >= 1, ErrorCode::InvalidConfig, "n_samples < 1");
    LIFT_BEMT_REQUIRE(!metrics.empty(), ErrorCode::InvalidConfig, "metrics empty");
    for (const auto& m : metrics) m.validate();
  }
};

struct BatchOutput final {
  ErrorCode code = ErrorCode::Ok;
  std::string message;
  std::vector<BemtOutput> samples;
  std::vector<std::pair<std::string, lift::stats::EmpiricalCDF>> dists;

  bool ok() const noexcept { return code == ErrorCode::Ok; }
};

inline bool extract_metric(const BemtOutput& o, const std::string& id, double& out_val) noexcept {
  if (id == "THRUST_N") { out_val = o.thrust_N; return true; }
  if (id == "TORQUE_NM") { out_val = o.torque_Nm; return true; }
  if (id == "POWER_W") { out_val = o.power_W; return true; }
  if (id == "FM") { out_val = o.FM; return true; }
  if (id == "PROP_EFF") { out_val = o.prop_eff; return true; }
  if (id == "CT") { out_val = o.Ct; return true; }
  if (id == "CQ") { out_val = o.Cq; return true; }
  if (id == "CP") { out_val = o.Cp; return true; }
  if (id == "RESIDUAL") { out_val = o.residual; return true; }
  if (id == "ITERS") { out_val = static_cast<double>(o.iters); return true; }
  return false;
}

inline lift::stats::EmpiricalCDF* find_or_create_ecdf(std::vector<std::pair<std::string, lift::stats::EmpiricalCDF>>& dists,
                                                      const std::string& id) {
  for (auto& p : dists) if (p.first == id) return &p.second;
  dists.emplace_back(id, lift::stats::EmpiricalCDF{});
  return &dists.back().second;
}

inline BatchOutput run_bemt_batch(const BemtCore& core, const BatchConfig& cfg_in, ISampleProvider& provider) {
  BatchConfig cfg = cfg_in;
  cfg.validate();

  BatchOutput out;
  out.samples.reserve(cfg.n_samples);
  out.dists.reserve(cfg.metrics.size());

  for (const auto& m : cfg.metrics) {
    auto* ecdf = find_or_create_ecdf(out.dists, m.id);
    ecdf->reserve(cfg.n_samples);
  }

  for (std::size_t i = 0; i < cfg.n_samples; ++i) {
    SampleInputs si;
    if (!provider.sample(i, si)) break;

    si.validate();
    if (!si.airfoils) {
      out.code = ErrorCode::InvalidInput;
      out.message = "airfoil database missing";
      break;
    }

    BemtOutput bo = core.evaluate(si.geom, *si.airfoils, si.env, si.op);
    out.samples.push_back(bo);

    const bool include = (cfg.policy == SamplePolicy::IncludeAll) ||
                         (cfg.policy == SamplePolicy::OnlyOk && bo.code == ErrorCode::Ok);
    if (!include) continue;

    for (const auto& m : cfg.metrics) {
      double v = 0.0;
      if (!extract_metric(bo, m.id, v)) continue;
      if (!lift::bemt::is_finite(v)) continue;
      auto* ecdf = find_or_create_ecdf(out.dists, m.id);
      ecdf->push(v);
    }
  }

  for (auto& p : out.dists) p.second.finalize();
  return out;
}

} // namespace lift::bemt

