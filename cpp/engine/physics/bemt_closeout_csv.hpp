/*
===============================================================================
Fragment 3.1.13 — Closeout Runner (Hover + Forward + Sensitivities + Disk Metrics) (C++)
File: bemt_closeout_csv.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/bemt_forward.hpp"
#include "engine/physics/bemt_metrics.hpp"
#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/bemt_require.hpp"
#include "engine/physics/bemt_sensitivity.hpp"
#include "engine/physics/bemt_types.hpp"
#include "engine/physics/bemt_solver.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::bemt {

// One closeout case definition.
// - hover_in is mandatory.
// - forward run optional.
// - sensitivity run optional.
struct CloseoutCase final {
    std::string case_id;

    BemtInputs hover_in;

    bool run_forward = false;
    double V_inplane_mps = 0.0;
    ForwardConfig forward_cfg{};

    bool run_sensitivity = false;
    SensitivityConfig sens_cfg{};
};

// Flat row for CSV output (kept stable for downstream scripts).
// This is the minimal closeout schema you can extend later.
struct CloseoutRow {
    std::string case_id;

    // Disk metrics
    double A_m2 = 0.0;
    double DL_N_m2 = 0.0;

    // Hover results
    ErrorCode hover_code = ErrorCode::Ok;
    double hover_T_N = 0.0;
    double hover_Q_Nm = 0.0;
    double hover_P_W = 0.0;
    double hover_vi_mps = 0.0;
    double hover_FM = 0.0;
    double hover_collective_rad = 0.0;
    std::size_t hover_inflow_iters = 0;
    std::size_t hover_trim_iters = 0;

    // Forward results (optional)
    ErrorCode fwd_code = ErrorCode::Ok;
    double V_inplane_mps = 0.0;
    double fwd_T_N = 0.0;
    double fwd_Q_Nm = 0.0;
    double fwd_P_W = 0.0;
    double fwd_vi_mps = 0.0;

    // Sensitivities (optional) normalized
    double sens_omega_n_dT = 0.0;
    double sens_omega_n_dP = 0.0;

    double sens_collective_n_dT = 0.0;
    double sens_collective_n_dP = 0.0;

    double sens_rho_n_dT = 0.0;
    double sens_rho_n_dP = 0.0;

    double sens_radius_n_dT = 0.0;
    double sens_radius_n_dP = 0.0;

    double sens_chord_n_dT = 0.0;
    double sens_chord_n_dP = 0.0;

    // Sizing factor hook
    double kT = 1.0;
};

class CloseoutRunner final {
public:
    explicit CloseoutRunner(const IAirfoilPolar& polar)
        : polar_(polar), hover_(polar), fwd_(polar), sens_(polar) {}

    std::vector<CloseoutRow> run(const std::vector<CloseoutCase>& cases,
                                double kT_for_sizing = 1.0) const;

private:
    static std::string esc_csv_(const std::string& s);

private:
    const IAirfoilPolar& polar_;
    BemtSolver hover_;
    BemtForwardSolver fwd_;
    SensitivityAnalyzer sens_;
};

// CSV writer
std::string closeout_csv_header();
std::string closeout_csv_row(const CloseoutRow& r);
std::string closeout_csv(const std::vector<CloseoutRow>& rows);

} // namespace lift::bemt
