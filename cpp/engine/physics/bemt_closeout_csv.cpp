#include "engine/physics/bemt_closeout_csv.hpp"

#include <iomanip>
#include <sstream>

namespace lift::bemt {

namespace {
double rotor_disk_area_m2(double radius_m) {
  if (!(is_finite(radius_m) && radius_m > 0.0)) return 0.0;
  return kPi * radius_m * radius_m;
}

std::string esc_csv(const std::string& s) {
  bool need = false;
  for (char c : s) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r') { need = true; break; }
  }
  if (!need) return s;

  std::string out;
  out.reserve(s.size() + 8);
  out.push_back('"');
  for (char c : s) {
    if (c == '"') out.append("\"\"");
    else out.push_back(c);
  }
  out.push_back('"');
  return out;
}

std::string ec_to_u16(ErrorCode c) {
  return std::to_string(static_cast<std::uint16_t>(c));
}
}  // namespace

std::vector<CloseoutRow> CloseoutRunner::run(const std::vector<CloseoutCase>& cases, double kT_for_sizing) const {
  LIFT_BEMT_REQUIRE(is_finite(kT_for_sizing) && kT_for_sizing > 0.0, ErrorCode::InvalidInput, "kT_for_sizing invalid");

  std::vector<CloseoutRow> rows;
  rows.reserve(cases.size());

  for (const auto& c : cases) {
    LIFT_BEMT_REQUIRE(!c.case_id.empty(), ErrorCode::InvalidInput, "CloseoutCase.case_id empty");
    c.hover_in.geom.validate();
    c.hover_in.env.validate();
    c.hover_in.op.validate();
    c.hover_in.cfg.validate();
    if (c.run_forward) {
      c.forward_cfg.validate();
      LIFT_BEMT_REQUIRE(is_finite(c.V_inplane_mps) && c.V_inplane_mps >= 0.0, ErrorCode::InvalidInput, "CloseoutCase.V_inplane_mps invalid");
    }
    if (c.run_sensitivity) {
      c.sens_cfg.validate();
    }

    CloseoutRow row{};
    row.case_id = c.case_id;
    row.kT = kT_for_sizing;

    // Hover solve
    const BemtResult hover = hover_.solve(c.hover_in);

    row.hover_code = hover.code;
    row.hover_T_N = hover.thrust_N;
    row.hover_Q_Nm = hover.torque_Nm;
    row.hover_P_W = hover.power_W;
    row.hover_vi_mps = hover.induced_velocity_m_s;
    row.hover_FM = hover.figure_of_merit;
    row.hover_collective_rad = hover.collective_offset_rad;
    row.hover_inflow_iters = hover.inflow_iters;
    row.hover_trim_iters = hover.trim_iters;

    // Disk metrics (area/disk loading; ideal power optional)
    const double A = rotor_disk_area_m2(c.hover_in.geom.radius_m);
    row.A_m2 = A;
    row.DL_N_m2 = (A > 0.0) ? safe_div(row.hover_T_N, A, 0.0) : 0.0;

    // Forward solve (optional)
    if (c.run_forward) {
      const auto fwd = fwd_.solve(c.hover_in.geom, c.hover_in.env, c.hover_in.op, c.hover_in.cfg,
                                  c.V_inplane_mps, c.forward_cfg);

      row.fwd_code = fwd.code;
      row.V_inplane_mps = c.V_inplane_mps;
      row.fwd_T_N = fwd.thrust_N;
      row.fwd_Q_Nm = fwd.torque_Nm;
      row.fwd_P_W = fwd.power_W;
      row.fwd_vi_mps = fwd.induced_velocity_mps;
    } else {
      row.fwd_code = ErrorCode::Ok;
    }

    // Sensitivities (optional)
    if (c.run_sensitivity && hover.code == ErrorCode::Ok) {
      auto sr = sens_.compute(c.hover_in, c.sens_cfg);

      row.sens_omega_n_dT = sr.omega.n_dT;
      row.sens_omega_n_dP = sr.omega.n_dP;

      row.sens_collective_n_dT = sr.collective.n_dT;
      row.sens_collective_n_dP = sr.collective.n_dP;

      row.sens_rho_n_dT = sr.rho_sens.n_dT;
      row.sens_rho_n_dP = sr.rho_sens.n_dP;

      row.sens_radius_n_dT = sr.radius_scale.n_dT;
      row.sens_radius_n_dP = sr.radius_scale.n_dP;

      row.sens_chord_n_dT = sr.chord_scale.n_dT;
      row.sens_chord_n_dP = sr.chord_scale.n_dP;
    }

    rows.push_back(std::move(row));
  }

  return rows;
}

std::string closeout_csv_header() {
  return
      "case_id,"
      "A_m2,DL_N_m2,"
      "hover_code,hover_T_N,hover_Q_Nm,hover_P_W,hover_vi_mps,hover_FM,hover_collective_rad,hover_inflow_iters,hover_trim_iters,"
      "fwd_code,V_inplane_mps,fwd_T_N,fwd_Q_Nm,fwd_P_W,fwd_vi_mps,"
      "sens_omega_n_dT,sens_omega_n_dP,"
      "sens_collective_n_dT,sens_collective_n_dP,"
      "sens_rho_n_dT,sens_rho_n_dP,"
      "sens_radius_n_dT,sens_radius_n_dP,"
      "sens_chord_n_dT,sens_chord_n_dP,"
      "kT\n";
}

std::string closeout_csv_row(const CloseoutRow& r) {
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os << std::setprecision(6);

  os
      << esc_csv(r.case_id) << ","
      << r.A_m2 << "," << r.DL_N_m2 << ","
      << ec_to_u16(r.hover_code) << ","
      << r.hover_T_N << "," << r.hover_Q_Nm << "," << r.hover_P_W << "," << r.hover_vi_mps << "," << r.hover_FM << "," << r.hover_collective_rad << ","
      << r.hover_inflow_iters << "," << r.hover_trim_iters << ","
      << ec_to_u16(r.fwd_code) << "," << r.V_inplane_mps << "," << r.fwd_T_N << "," << r.fwd_Q_Nm << "," << r.fwd_P_W << "," << r.fwd_vi_mps << ","
      << r.sens_omega_n_dT << "," << r.sens_omega_n_dP << ","
      << r.sens_collective_n_dT << "," << r.sens_collective_n_dP << ","
      << r.sens_rho_n_dT << "," << r.sens_rho_n_dP << ","
      << r.sens_radius_n_dT << "," << r.sens_radius_n_dP << ","
      << r.sens_chord_n_dT << "," << r.sens_chord_n_dP << ","
      << r.kT
      << "\n";

  return os.str();
}

std::string closeout_csv(const std::vector<CloseoutRow>& rows) {
  std::string out;
  out.reserve(256 + rows.size() * 256);
  out.append(closeout_csv_header());
  for (const auto& r : rows) out.append(closeout_csv_row(r));
  return out;
}

}  // namespace lift::bemt
