/*
===============================================================================
Fragment 3.1.53 — BEMT Core Implementation (Hover + Forward, Induction Iteration, Tip Loss, Outputs/FM) (C++)
File: cpp/engine/physics/bemt_core.hpp
===============================================================================
*/

#pragma once

#include "engine/physics/airfoil_polar.hpp"
#include "engine/physics/bemt_adapters.hpp"
#include "engine/physics/bemt_types.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lift::bemt {

struct CoreConfig final {
    std::size_t max_iter = 80;
    double tol = 1e-4;
    double relaxation = 0.35;
    bool use_prandtl_tip_loss = true;

    double mach_max = 0.0;       // <=0 disables
    double reynolds_min = 0.0;   // <=0 disables
    double reynolds_max = 0.0;   // <=0 disables
    double min_tip_loss_F = 1e-3;

    std::string default_airfoil_id; // optional fallback if station airfoil_id empty

    void validate() const {
        LIFT_BEMT_REQUIRE(max_iter >= 1 && max_iter <= 100000, ErrorCode::InvalidConfig, "CoreConfig.max_iter invalid");
        LIFT_BEMT_REQUIRE(is_finite(tol) && tol > 0.0 && tol < 1.0, ErrorCode::InvalidConfig, "CoreConfig.tol invalid");
        LIFT_BEMT_REQUIRE(is_finite(relaxation) && relaxation > 0.0 && relaxation <= 1.0, ErrorCode::InvalidConfig, "CoreConfig.relaxation invalid");
        LIFT_BEMT_REQUIRE(is_finite(mach_max) && mach_max >= 0.0, ErrorCode::InvalidConfig, "CoreConfig.mach_max invalid");
        LIFT_BEMT_REQUIRE(is_finite(reynolds_min) && reynolds_min >= 0.0, ErrorCode::InvalidConfig, "CoreConfig.reynolds_min invalid");
        LIFT_BEMT_REQUIRE(is_finite(reynolds_max) && reynolds_max >= 0.0, ErrorCode::InvalidConfig, "CoreConfig.reynolds_max invalid");
        LIFT_BEMT_REQUIRE(is_finite(min_tip_loss_F) && min_tip_loss_F > 0.0 && min_tip_loss_F <= 1.0,
                          ErrorCode::InvalidConfig, "CoreConfig.min_tip_loss_F invalid");
    }
};

struct SectionOutput final {
    double r_m = 0.0;
    double phi_rad = 0.0;
    double alpha_rad = 0.0;
    double cl = 0.0;
    double cd = 0.0;
    double dT_N = 0.0;
    double dQ_Nm = 0.0;
    double reynolds = 0.0;
    double mach = 0.0;
};

struct BemtOutput final {
    ErrorCode code = ErrorCode::Ok;
    std::string message;

    std::vector<SectionOutput> sections;

    double thrust_N = 0.0;
    double torque_Nm = 0.0;
    double power_W = 0.0;

    double Ct = 0.0;
    double Cq = 0.0;
    double Cp = 0.0;

    double FM = 0.0;
    double prop_eff = 0.0;

    double residual = 0.0;
    std::size_t iters = 0;
};

class BemtCore final {
public:
    explicit BemtCore(CoreConfig cfg = {}) : cfg_(std::move(cfg)) { cfg_.validate(); }

    BemtOutput evaluate(const RotorGeometry& geom,
                        const IAirfoilDatabase& airfoils,
                        const Environment& env,
                        const OperatingPoint& op) const;

private:
    CoreConfig cfg_{};
};

} // namespace lift::bemt

