/*
===============================================================================
Fragment 3.1.40 — Structural & Gearbox Feasibility Screen (Mast Bending/Torque, Backlash, Bearings, Housing Mass) (C++)
File: cpp/engine/structures/gearbox_feasibility.hpp
===============================================================================
*/

#pragma once

#include "../physics/bemt_require.hpp"
#include "../physics/bemt_safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::structures {

// -----------------------------
// Material + section models
// -----------------------------
struct IsoMaterial final {
  std::string name;
  double E_Pa = 0.0;      // Young's modulus
  double G_Pa = 0.0;      // Shear modulus
  double rho_kg_m3 = 0.0; // density
  double yield_Pa = 0.0;  // yield/allowable stress basis

  void validate() const {
    LIFT_BEMT_REQUIRE(!name.empty(), lift::bemt::ErrorCode::InvalidInput, "IsoMaterial.name empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(E_Pa) && E_Pa > 1e6, lift::bemt::ErrorCode::InvalidInput, "E invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(G_Pa) && G_Pa > 1e6, lift::bemt::ErrorCode::InvalidInput, "G invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(rho_kg_m3) && rho_kg_m3 > 0.0, lift::bemt::ErrorCode::InvalidInput, "rho invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(yield_Pa) && yield_Pa > 1e6, lift::bemt::ErrorCode::InvalidInput, "yield invalid");
  }
};

struct TubeSection final {
  double L_m = 0.0;
  double OD_m = 0.0;
  double t_m = 0.0; // wall thickness

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(L_m) && L_m > 0.0, lift::bemt::ErrorCode::InvalidInput, "TubeSection.L invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(OD_m) && OD_m > 0.0, lift::bemt::ErrorCode::InvalidInput, "TubeSection.OD invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(t_m) && t_m > 0.0, lift::bemt::ErrorCode::InvalidInput, "TubeSection.t invalid");
    LIFT_BEMT_REQUIRE(2.0 * t_m < OD_m, lift::bemt::ErrorCode::InvalidInput, "TubeSection.t too large");
  }

  double ID_m() const noexcept { return std::max(0.0, OD_m - 2.0 * t_m); }

  double I_m4() const noexcept {
    const double OD = OD_m;
    const double ID = ID_m();
    const double I = (M_PI / 64.0) * (std::pow(OD, 4) - std::pow(ID, 4));
    return (lift::bemt::is_finite(I) && I >= 0.0) ? I : 0.0;
  }

  double J_m4() const noexcept {
    const double OD = OD_m;
    const double ID = ID_m();
    const double J = (M_PI / 32.0) * (std::pow(OD, 4) - std::pow(ID, 4));
    return (lift::bemt::is_finite(J) && J >= 0.0) ? J : 0.0;
  }

  double area_m2() const noexcept {
    const double OD = OD_m;
    const double ID = ID_m();
    const double A = (M_PI / 4.0) * (OD * OD - ID * ID);
    return (lift::bemt::is_finite(A) && A >= 0.0) ? A : 0.0;
  }

  double mass_kg(const IsoMaterial& mat) const noexcept {
    const double m = area_m2() * L_m * mat.rho_kg_m3;
    return (lift::bemt::is_finite(m) && m >= 0.0) ? m : 0.0;
  }
};

// -----------------------------
// Driveline models
// -----------------------------
struct MastLoadCase final {
  double tip_force_N = 0.0;   // lateral equivalent load at tip
  double tip_moment_Nm = 0.0; // bending moment at tip (if known)
  double torque_Nm = 0.0;     // torsion from rotor torque

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(tip_force_N) && tip_force_N >= 0.0, lift::bemt::ErrorCode::InvalidInput, "tip_force invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(tip_moment_Nm) && tip_moment_Nm >= 0.0, lift::bemt::ErrorCode::InvalidInput, "tip_moment invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(torque_Nm) && torque_Nm >= 0.0, lift::bemt::ErrorCode::InvalidInput, "torque invalid");
  }
};

struct GearboxSpec final {
  std::string id;
  double ratio = 1.0;
  double eta = 0.97;
  double backlash_deg = 0.0;
  double housing_volume_m3 = 0.0;
  double housing_rho_kg_m3 = 2700.0;

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "GearboxSpec.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(ratio) && ratio >= 0.2 && ratio <= 50.0, lift::bemt::ErrorCode::InvalidInput, "ratio invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(eta) && eta > 0.0 && eta <= 1.0, lift::bemt::ErrorCode::InvalidInput, "eta invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(backlash_deg) && backlash_deg >= 0.0 && backlash_deg <= 10.0,
                      lift::bemt::ErrorCode::InvalidInput, "backlash invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(housing_volume_m3) && housing_volume_m3 >= 0.0,
                      lift::bemt::ErrorCode::InvalidInput, "housing_volume invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(housing_rho_kg_m3) && housing_rho_kg_m3 > 0.0,
                      lift::bemt::ErrorCode::InvalidInput, "housing_rho invalid");
  }

  double housing_mass_kg(double default_mass_kg = 0.0) const noexcept {
    if (housing_volume_m3 > 0.0) {
      const double m = housing_volume_m3 * housing_rho_kg_m3;
      return (lift::bemt::is_finite(m) && m >= 0.0) ? m : 0.0;
    }
    return (lift::bemt::is_finite(default_mass_kg) && default_mass_kg >= 0.0) ? default_mass_kg : 0.0;
  }
};

struct BearingSpec final {
  std::string id;
  double C_N = 0.0;
  double P_N = 0.0;
  double p = 3.0; // default ball bearing

  void validate() const {
    LIFT_BEMT_REQUIRE(!id.empty(), lift::bemt::ErrorCode::InvalidInput, "BearingSpec.id empty");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(C_N) && C_N > 0.0, lift::bemt::ErrorCode::InvalidInput, "C_N invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(P_N) && P_N > 0.0, lift::bemt::ErrorCode::InvalidInput, "P_N invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(p) && p >= 2.0 && p <= 4.0, lift::bemt::ErrorCode::InvalidInput, "p invalid");
  }

  double life_ratio() const noexcept {
    const double r = std::pow(C_N / P_N, p);
    return (lift::bemt::is_finite(r) && r >= 0.0) ? r : 0.0;
  }
};

// -----------------------------
// Config thresholds (GO/NO-GO)
// -----------------------------
struct GearboxFeasibilityConfig final {
  double sigma_allow_frac = 0.50;
  double tau_allow_frac = 0.50;
  double tip_deflection_max_m = 0.0; // <=0 disables
  double twist_max_deg = 0.0;        // <=0 disables
  double backlash_max_deg = 0.0;     // <=0 disables
  double bearing_life_ratio_min = 0.0; // <=0 disables
  double housing_mass_max_kg = 0.0;    // <=0 disables

  void validate() const {
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(sigma_allow_frac) && sigma_allow_frac > 0.0 && sigma_allow_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "sigma_allow_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(tau_allow_frac) && tau_allow_frac > 0.0 && tau_allow_frac <= 1.0,
                      lift::bemt::ErrorCode::InvalidConfig, "tau_allow_frac invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(tip_deflection_max_m) && tip_deflection_max_m >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "tip_deflection_max invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(twist_max_deg) && twist_max_deg >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "twist_max_deg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(backlash_max_deg) && backlash_max_deg >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "backlash_max_deg invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(bearing_life_ratio_min) && bearing_life_ratio_min >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "bearing_life_ratio_min invalid");
    LIFT_BEMT_REQUIRE(lift::bemt::is_finite(housing_mass_max_kg) && housing_mass_max_kg >= 0.0,
                      lift::bemt::ErrorCode::InvalidConfig, "housing_mass_max invalid");
  }
};

// -----------------------------
// Outputs
// -----------------------------
enum class FeasVerdict : std::uint8_t { Pass = 0, Fail = 1, Unknown = 2 };

struct FeasCheck final {
  std::string id;
  FeasVerdict verdict = FeasVerdict::Unknown;
  double value = 0.0;
  double threshold = 0.0;
  std::string message;
};

struct GearboxFeasibilityMetrics final {
  double M_bend_Nm = 0.0;
  double sigma_bend_Pa = 0.0;
  double sigma_allow_Pa = 0.0;
  double tip_deflection_m = 0.0;

  double T_torque_Nm = 0.0;
  double tau_torsion_Pa = 0.0;
  double tau_allow_Pa = 0.0;
  double twist_deg = 0.0;

  double bearing_life_ratio = 0.0;

  double backlash_deg = 0.0;
  double housing_mass_kg = 0.0;

  double mast_mass_kg = 0.0;
};

struct GearboxFeasibilityReport final {
  lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
  std::vector<FeasCheck> checks;

  bool ok() const noexcept {
    if (code != lift::bemt::ErrorCode::Ok) return false;
    for (const auto& c : checks)
      if (c.verdict == FeasVerdict::Fail) return false;
    return true;
  }
};

struct GearboxFeasibilityOut final {
  GearboxFeasibilityMetrics metrics{};
  GearboxFeasibilityReport report{};
};

// -----------------------------
// Mechanics helpers
// -----------------------------
inline double bending_moment_at_base(const TubeSection& tube, const MastLoadCase& lc) noexcept {
  if (lc.tip_moment_Nm > 0.0) return lc.tip_moment_Nm;
  return lc.tip_force_N * tube.L_m;
}

inline double bending_stress_Pa(const TubeSection& tube, double M_Nm) noexcept {
  const double I = tube.I_m4();
  const double c = 0.5 * tube.OD_m;
  if (I <= 0.0 || c <= 0.0) return 0.0;
  const double sigma = (M_Nm * c) / I;
  return (lift::bemt::is_finite(sigma) && sigma >= 0.0) ? sigma : 0.0;
}

inline double tip_deflection_cantilever_m(const TubeSection& tube, const IsoMaterial& mat, double F_N) noexcept {
  const double I = tube.I_m4();
  if (mat.E_Pa <= 0.0 || I <= 0.0) return 0.0;
  const double L = tube.L_m;
  const double delta = (F_N * L * L * L) / (3.0 * mat.E_Pa * I);
  return (lift::bemt::is_finite(delta) && delta >= 0.0) ? delta : 0.0;
}

inline double torsional_shear_Pa(const TubeSection& tube, double T_Nm) noexcept {
  const double J = tube.J_m4();
  const double r = 0.5 * tube.OD_m;
  if (J <= 0.0 || r <= 0.0) return 0.0;
  const double tau = (T_Nm * r) / J;
  return (lift::bemt::is_finite(tau) && tau >= 0.0) ? tau : 0.0;
}

inline double torsional_twist_deg(const TubeSection& tube, const IsoMaterial& mat, double T_Nm) noexcept {
  const double J = tube.J_m4();
  if (mat.G_Pa <= 0.0 || J <= 0.0) return 0.0;
  const double theta_rad = (T_Nm * tube.L_m) / (mat.G_Pa * J);
  const double deg = theta_rad * 57.29577951308232;
  return (lift::bemt::is_finite(deg) && deg >= 0.0) ? deg : 0.0;
}

inline FeasCheck check_leq(const std::string& id, double val, double thr, const std::string& msg) {
  if (!lift::bemt::is_finite(val) || !lift::bemt::is_finite(thr) || thr <= 0.0)
    return {id, FeasVerdict::Unknown, val, thr, "disabled/invalid"};
  return {id, (val <= thr ? FeasVerdict::Pass : FeasVerdict::Fail), val, thr, (val <= thr ? "" : msg)};
}
inline FeasCheck check_geq(const std::string& id, double val, double thr, const std::string& msg) {
  if (!lift::bemt::is_finite(val) || !lift::bemt::is_finite(thr) || thr <= 0.0)
    return {id, FeasVerdict::Unknown, val, thr, "disabled/invalid"};
  return {id, (val >= thr ? FeasVerdict::Pass : FeasVerdict::Fail), val, thr, (val >= thr ? "" : msg)};
}

// -----------------------------
// Main evaluation
// -----------------------------
inline GearboxFeasibilityOut evaluate_gearbox_feasibility(const TubeSection& mast_in,
                                                          const IsoMaterial& mast_mat_in,
                                                          const MastLoadCase& load_in,
                                                          const GearboxSpec* gearbox,   // optional
                                                          const BearingSpec* bearing,   // optional
                                                          const GearboxFeasibilityConfig& cfg_in,
                                                          double default_housing_mass_kg = 0.0) {
  TubeSection mast = mast_in;
  mast.validate();

  IsoMaterial mast_mat = mast_mat_in;
  mast_mat.validate();

  MastLoadCase load = load_in;
  load.validate();

  GearboxFeasibilityConfig cfg = cfg_in;
  cfg.validate();

  GearboxFeasibilityOut out;

  out.metrics.sigma_allow_Pa = cfg.sigma_allow_frac * mast_mat.yield_Pa;
  out.metrics.tau_allow_Pa = cfg.tau_allow_frac * mast_mat.yield_Pa;
  out.metrics.mast_mass_kg = mast.mass_kg(mast_mat);

  out.metrics.M_bend_Nm = bending_moment_at_base(mast, load);
  out.metrics.sigma_bend_Pa = bending_stress_Pa(mast, out.metrics.M_bend_Nm);
  out.metrics.tip_deflection_m = tip_deflection_cantilever_m(mast, mast_mat, load.tip_force_N);

  out.metrics.T_torque_Nm = load.torque_Nm;
  out.metrics.tau_torsion_Pa = torsional_shear_Pa(mast, out.metrics.T_torque_Nm);
  out.metrics.twist_deg = torsional_twist_deg(mast, mast_mat, out.metrics.T_torque_Nm);

  out.report.checks.push_back(check_leq(
      "STRUCT.MAST.BENDING_STRESS_MAX",
      out.metrics.sigma_bend_Pa,
      out.metrics.sigma_allow_Pa,
      "mast bending stress exceeds allowable"));
  out.report.checks.push_back(check_leq(
      "STRUCT.MAST.TORSION_SHEAR_MAX",
      out.metrics.tau_torsion_Pa,
      out.metrics.tau_allow_Pa,
      "mast torsion shear exceeds allowable"));

  if (cfg.tip_deflection_max_m > 0.0) {
    out.report.checks.push_back(check_leq(
        "STRUCT.MAST.TIP_DEFLECTION_MAX",
        out.metrics.tip_deflection_m,
        cfg.tip_deflection_max_m,
        "mast tip deflection exceeds limit"));
  } else {
    out.report.checks.push_back({"STRUCT.MAST.TIP_DEFLECTION_MAX", FeasVerdict::Unknown,
                                 out.metrics.tip_deflection_m, cfg.tip_deflection_max_m, "disabled"});
  }

  if (cfg.twist_max_deg > 0.0) {
    out.report.checks.push_back(check_leq(
        "STRUCT.MAST.TWIST_MAX",
        out.metrics.twist_deg,
        cfg.twist_max_deg,
        "mast torsional twist exceeds limit"));
  } else {
    out.report.checks.push_back({"STRUCT.MAST.TWIST_MAX", FeasVerdict::Unknown,
                                 out.metrics.twist_deg, cfg.twist_max_deg, "disabled"});
  }

  if (bearing) {
    bearing->validate();
    out.metrics.bearing_life_ratio = bearing->life_ratio();
    if (cfg.bearing_life_ratio_min > 0.0) {
      out.report.checks.push_back(check_geq(
          "STRUCT.BEARING.LIFE_RATIO_MIN",
          out.metrics.bearing_life_ratio,
          cfg.bearing_life_ratio_min,
          "bearing life ratio below minimum"));
    } else {
      out.report.checks.push_back({"STRUCT.BEARING.LIFE_RATIO_MIN", FeasVerdict::Unknown,
                                   out.metrics.bearing_life_ratio, cfg.bearing_life_ratio_min, "disabled"});
    }
  } else {
    out.metrics.bearing_life_ratio = 0.0;
    out.report.checks.push_back({"STRUCT.BEARING.LIFE_RATIO_MIN", FeasVerdict::Unknown, 0.0, cfg.bearing_life_ratio_min, "no bearing model"});
  }

  if (gearbox) {
    gearbox->validate();
    out.metrics.backlash_deg = gearbox->backlash_deg;
    out.metrics.housing_mass_kg = gearbox->housing_mass_kg(default_housing_mass_kg);

    if (cfg.backlash_max_deg > 0.0) {
      out.report.checks.push_back(check_leq(
          "GEARBOX.BACKLASH_MAX",
          out.metrics.backlash_deg,
          cfg.backlash_max_deg,
          "gearbox backlash exceeds sync limit"));
    } else {
      out.report.checks.push_back({"GEARBOX.BACKLASH_MAX", FeasVerdict::Unknown,
                                   out.metrics.backlash_deg, cfg.backlash_max_deg, "disabled"});
    }

    if (cfg.housing_mass_max_kg > 0.0) {
      out.report.checks.push_back(check_leq(
          "GEARBOX.HOUSING_MASS_MAX",
          out.metrics.housing_mass_kg,
          cfg.housing_mass_max_kg,
          "gearbox housing mass exceeds limit"));
    } else {
      out.report.checks.push_back({"GEARBOX.HOUSING_MASS_MAX", FeasVerdict::Unknown,
                                   out.metrics.housing_mass_kg, cfg.housing_mass_max_kg, "disabled"});
    }
  } else {
    out.metrics.backlash_deg = 0.0;
    out.metrics.housing_mass_kg = 0.0;
    out.report.checks.push_back({"GEARBOX.BACKLASH_MAX", FeasVerdict::Unknown, 0.0, cfg.backlash_max_deg, "no gearbox model"});
    out.report.checks.push_back({"GEARBOX.HOUSING_MASS_MAX", FeasVerdict::Unknown, 0.0, cfg.housing_mass_max_kg, "no gearbox model"});
  }

  out.report.code = lift::bemt::ErrorCode::Ok;
  return out;
}

} // namespace lift::structures
