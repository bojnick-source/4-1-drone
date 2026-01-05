/*
===============================================================================
Fragment 3.2.04 — Corrected Closeout CSV (Adds corrected_* columns)
File: cfd_closeout_csv.cpp
===============================================================================
*/

#include "cfd_closeout_csv.hpp"

#include <sstream>
#include <iomanip>

namespace lift::bemt {

namespace {
static std::string esc_csv(const std::string& s) {
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

} // namespace

std::string closeout_corrected_csv_header() {
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
        "kT,"
        "cfd_corr_T,cfd_corr_P,"
        "corr_hover_T_N,corr_hover_P_W,"
        "corr_fwd_T_N,corr_fwd_P_W\n";
}

std::string closeout_corrected_csv_row(const CloseoutRowCorrected& r) {
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(8);
    os
        << esc_csv(r.case_id) << ","
        << r.A_m2 << "," << r.DL_N_m2 << ","
        << static_cast<unsigned>(r.hover_code) << ","
        << r.hover_T_N << "," << r.hover_Q_Nm << "," << r.hover_P_W << "," << r.hover_vi_mps << "," << r.hover_FM << "," << r.hover_collective_rad << ","
        << r.hover_inflow_iters << "," << r.hover_trim_iters << ","
        << static_cast<unsigned>(r.fwd_code) << "," << r.V_inplane_mps << "," << r.fwd_T_N << "," << r.fwd_Q_Nm << "," << r.fwd_P_W << "," << r.fwd_vi_mps << ","
        << r.sens_omega_n_dT << "," << r.sens_omega_n_dP << ","
        << r.sens_collective_n_dT << "," << r.sens_collective_n_dP << ","
        << r.sens_rho_n_dT << "," << r.sens_rho_n_dP << ","
        << r.sens_radius_n_dT << "," << r.sens_radius_n_dP << ","
        << r.sens_chord_n_dT << "," << r.sens_chord_n_dP << ","
        << r.kT << ","
        << r.cfd_corr_T << "," << r.cfd_corr_P << ","
        << r.corr_hover_T_N << "," << r.corr_hover_P_W << ","
        << r.corr_fwd_T_N << "," << r.corr_fwd_P_W
        << "\n";
    return os.str();
}

std::string closeout_corrected_csv(const std::vector<CloseoutRowCorrected>& rows) {
    std::string out;
    out.reserve(256 + rows.size() * 320);
    out.append(closeout_corrected_csv_header());
    for (const auto& r : rows) out.append(closeout_corrected_csv_row(r));
    return out;
}

} // namespace lift::bemt
