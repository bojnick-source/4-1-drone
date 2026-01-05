/*
===============================================================================
Fragment 3.1.60 — BEMT + Closeout Wiring Example (Evaluator → Evidence + Gate Thresholds) (C++)
File: cpp/engine/closeout/bemt_closeout_bridge.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_core.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"
#include "../bemt/bemt_metrics.hpp"

#include "closeout_pipeline.hpp"
#include "go_nogo_thresholds.hpp"

#include "../compliance/rules_verification.hpp"

#include <string>
#include <memory>

namespace lift::closeout::bridge {

struct BemtCloseoutInputs final {
  lift::bemt::RotorGeometry geom;
  std::shared_ptr<const lift::bemt::IAirfoilDatabase> airfoils;
  lift::bemt::Environment env;
  lift::bemt::OperatingPoint op;

  void validate() const {
    geom.validate();
    env.validate();
    op.validate();
    LIFT_BEMT_REQUIRE(static_cast<bool>(airfoils), lift::bemt::ErrorCode::InvalidInput, "airfoils missing");
  }
};

inline lift::closeout::CloseoutOutput run_bemt_closeout(
    const lift::bemt::BemtCore& core,
    const BemtCloseoutInputs& in,
    const lift::closeout::GoNoGoThresholds& thr,
    bool build_compliance_evidence = true) {
  in.validate();
  thr.validate();

  lift::closeout::CloseoutOutput co;

  const lift::bemt::BemtOutput bo = core.evaluate(in.geom, *in.airfoils, in.env, in.op);

  // Evidence and KV mirrors
  lift::bemt::metrics::append_bemt_evidence(co.evidence, in.geom, in.env, in.op, bo, "bemt");
  lift::bemt::metrics::append_bemt_kv(co.export_kv, in.geom, in.env, in.op, bo, "bemt");

  // Optional compliance placeholder (no clause mapping here)
  co.has_compliance = build_compliance_evidence;
  if (co.has_compliance) {
    co.compliance.code = lift::bemt::ErrorCode::Ok;
  }

  // Gate checks
  lift::closeout::GateReport gr;
  gr.code = bo.code;
  gr.verdict = lift::closeout::Verdict::Go;

  auto add_check = [&](const std::string& id, bool pass, double value, double threshold, const std::string& note) {
    lift::closeout::GateCheck c;
    c.id = id;
    c.pass = pass;
    c.value = value;
    c.threshold = threshold;
    c.note = note;
    c.validate();
    gr.checks.push_back(std::move(c));
    if (!pass) gr.verdict = lift::closeout::Verdict::NoGo;
  };

  add_check("BEMT.OK", bo.code == lift::bemt::ErrorCode::Ok, bo.code == lift::bemt::ErrorCode::Ok ? 1.0 : 0.0, 1.0, bo.message);
  add_check("BEMT.THRUST_MIN_N", bo.thrust_N >= thr.min_thrust_N, bo.thrust_N, thr.min_thrust_N, "thrust gate");
  add_check("BEMT.POWER_MAX_W", bo.power_W <= thr.max_power_W, bo.power_W, thr.max_power_W, "power gate");
  add_check("BEMT.RESIDUAL_MAX", bo.residual <= thr.max_residual, bo.residual, thr.max_residual, "convergence gate");

  const double area = lift::bemt::metrics::disk_area_m2(in.geom);
  add_check("BEMT.AREA_MIN_M2", area >= thr.min_disk_area_m2, area, thr.min_disk_area_m2, "disk area gate");

  if (thr.min_FM > 0.0 && in.op.mode == lift::bemt::OperatingPoint::FlightMode::Hover && std::abs(in.op.V_inf) <= 1e-6) {
    add_check("BEMT.FM_MIN", bo.FM >= thr.min_FM, bo.FM, thr.min_FM, "hover FM gate");
  }

  co.gate = std::move(gr);
  return co;
}

} // namespace lift::closeout::bridge
