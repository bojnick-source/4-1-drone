/*
===============================================================================
Fragment 3.1.55 — BEMT Regression & Sanity Test Vectors (Deterministic, No External Test Framework) (C++)
File: cpp/engine/bemt/bemt_validation.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_core.hpp"
#include "../physics/airfoil_polar.hpp"
#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace lift::bemt::validate {

struct CheckResult final {
  std::string id;
  bool pass = false;
  std::string note;
};

struct Report final {
  ErrorCode code = ErrorCode::Ok;
  std::string message;
  bool all_pass = true;
  std::vector<CheckResult> checks;

  void add(std::string id, bool pass, std::string note = {}) {
    checks.push_back(CheckResult{std::move(id), pass, std::move(note)});
    if (!pass) all_pass = false;
  }
};

class StaticAirfoilDatabase final : public IAirfoilDatabase {
public:
  explicit StaticAirfoilDatabase(std::shared_ptr<const IAirfoilPolar> polar) : polar_(std::move(polar)) {}
  std::shared_ptr<const IAirfoilPolar> get_polar(const PolarRequest& /*req*/) const override { return polar_; }

private:
  std::shared_ptr<const IAirfoilPolar> polar_;
};

inline std::shared_ptr<const IAirfoilPolar> make_toy_polar() {
  LinearPolarParams p;
  p.cl0 = 0.0;
  p.cla_per_rad = 2.0 * kPi;
  p.cd0 = 0.01;
  p.k = 0.02;
  p.aoa_stall_rad = deg2rad(20.0);
  p.cl_min = -4.0;
  p.cl_max = 4.0;
  p.cd_max = 4.0;
  p.validate();
  return std::make_shared<LinearPolar>(p);
}

inline RotorGeometry make_toy_rotor(std::uint32_t blades,
                                    double hub_m,
                                    double tip_m,
                                    double chord_m,
                                    double twist_rad) {
  RotorGeometry g;
  g.blade_count = blades;
  g.radius_m = tip_m;
  g.hub_radius_m = hub_m;
  g.tip_loss = TipLossModel::Prandtl;

  const int N = 21;
  g.stations.reserve(static_cast<std::size_t>(N));
  for (int i = 0; i < N; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(N - 1);
    const double r = hub_m + t * (tip_m - hub_m);
    BladeStation st;
    st.r_m = r;
    st.chord_m = chord_m;
    st.twist_rad = twist_rad;
    g.stations.push_back(st);
  }

  g.validate();
  return g;
}

inline Environment make_std_env() {
  Environment e;
  e.rho = 1.225;
  e.mu = 1.81e-5;
  e.a_m_s = 340.0;
  e.validate();
  return e;
}

inline bool finite_nonneg(double x) noexcept { return is_finite(x) && x >= 0.0; }

inline Report run_bemt_self_tests(const CoreConfig& cfg_in) {
  Report rep;

  CoreConfig cfg = cfg_in;
  cfg.validate();

  auto polar = make_toy_polar();
  StaticAirfoilDatabase db(polar);
  const auto env = make_std_env();
  const auto geom = make_toy_rotor(3, 0.05, 0.50, 0.06, deg2rad(10.0));

  BemtCore core(cfg);

  OperatingPoint opA;
  opA.mode = OperatingPoint::FlightMode::Hover;
  opA.omega_rad_s = 300.0;
  opA.collective_offset_rad = deg2rad(2.0);
  opA.V_inf = 0.0;
  opA.inflow_angle_rad = 0.0;
  opA.validate();

  const BemtOutput outA = core.evaluate(geom, db, env, opA);
  rep.add("A.ok", outA.code == ErrorCode::Ok, outA.message);
  rep.add("A.thrust_pos", outA.thrust_N > 0.0, "expected positive thrust");
  rep.add("A.power_pos", outA.power_W > 0.0, "expected positive power");
  rep.add("A.FM_range", is_finite(outA.FM) && outA.FM >= 0.0 && outA.FM <= 1.2, "FM out of range");

  OperatingPoint opB = opA;
  opB.omega_rad_s = 360.0;
  opB.validate();
  const BemtOutput outB = core.evaluate(geom, db, env, opB);
  rep.add("B.thrust_increase", outB.thrust_N > outA.thrust_N, "thrust should rise with omega");
  rep.add("B.power_increase", outB.power_W > outA.power_W, "power should rise with omega");

  OperatingPoint opC = opA;
  opC.collective_offset_rad = deg2rad(4.0);
  opC.validate();
  const BemtOutput outC = core.evaluate(geom, db, env, opC);
  rep.add("C.thrust_increase_collective", outC.thrust_N > outA.thrust_N, "thrust should rise with collective");

  OperatingPoint opD = opA;
  opD.mode = OperatingPoint::FlightMode::Forward;
  opD.V_inf = 10.0;
  opD.inflow_angle_rad = 0.0;
  opD.validate();
  const BemtOutput outD = core.evaluate(geom, db, env, opD);
  rep.add("D.power_nonneg", finite_nonneg(outD.power_W), "power must be finite/non-negative");
  rep.add("D.thrust_finite", is_finite(outD.thrust_N), "");

  rep.add("Residual.A", is_finite(outA.residual), "");
  rep.add("Residual.B", is_finite(outB.residual), "");
  rep.add("Residual.C", is_finite(outC.residual), "");
  rep.add("Residual.D", is_finite(outD.residual), "");

  if (!rep.all_pass) {
    rep.code = ErrorCode::NumericalError;
    rep.message = "BEMT self-tests failed";
  }
  return rep;
}

} // namespace lift::bemt::validate

