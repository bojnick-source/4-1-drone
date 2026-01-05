/*
===============================================================================
Fragment 3.2.01 — CFD Job Manifest Generator (Top-N Export + Schema-Stable JSON/CSV)
File: cfd_manifest.hpp
===============================================================================
*/

#pragma once
#include "bemt_closeout_csv.hpp"
#include "closeout_thresholds.hpp"
#include "bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
#include <cmath>

namespace lift::bemt {

// CFD fidelity tiers: keep simple and stable.
enum class CfdTier : std::uint8_t {
    CFD0_ActuatorDisk = 0,   // cheap interference / net thrust-power trend
    CFD0_ActuatorLine = 1,   // still cheap, more structure
    CFD1_ResolvedBlades = 2  // expensive, final verification only
};

struct CfdJob final {
    std::string job_id;          // unique ID (stable across reruns if possible)
    std::string case_id;         // maps back to BEMT closeout
    CfdTier tier = CfdTier::CFD0_ActuatorDisk;

    // External references (paths/URIs) - optional, runner-defined.
    std::string geometry_ref;    // e.g., "exports/case123/rotor.step"
    std::string mesh_ref;        // optional if you pre-mesh

    // Operating point (minimal)
    double omega_rad_s = 0.0;
    double V_axial_mps = 0.0;    // positive down through disk
    double V_inplane_mps = 0.0;  // forward/side inflow magnitude

    // Environment
    double rho = 0.0;
    double mu = 0.0;

    // Expectations (from BEMT) for validation windows
    double bemt_T_N = 0.0;
    double bemt_P_W = 0.0;

    // Optional calibration knobs placeholders (filled later by results ingestor)
    double correction_thrust = 1.0;  // multiply BEMT thrust
    double correction_power = 1.0;   // multiply BEMT power

    void validate() const {
        LIFT_BEMT_REQUIRE(!job_id.empty(), ErrorCode::InvalidInput, "CfdJob.job_id empty");
        LIFT_BEMT_REQUIRE(!case_id.empty(), ErrorCode::InvalidInput, "CfdJob.case_id empty");

        LIFT_BEMT_REQUIRE(is_finite(omega_rad_s) && omega_rad_s > 0.0 && omega_rad_s < 1e6, ErrorCode::InvalidInput, "omega invalid");
        LIFT_BEMT_REQUIRE(is_finite(V_axial_mps) && std::abs(V_axial_mps) < 500.0, ErrorCode::InvalidInput, "V_axial invalid");
        LIFT_BEMT_REQUIRE(is_finite(V_inplane_mps) && V_inplane_mps >= 0.0 && V_inplane_mps < 500.0, ErrorCode::InvalidInput, "V_inplane invalid");

        LIFT_BEMT_REQUIRE(is_finite(rho) && rho > 0.0 && rho < 10.0, ErrorCode::InvalidInput, "rho invalid");
        LIFT_BEMT_REQUIRE(is_finite(mu) && mu > 0.0 && mu < 1.0, ErrorCode::InvalidInput, "mu invalid");

        LIFT_BEMT_REQUIRE(is_finite(bemt_T_N) && bemt_T_N >= 0.0, ErrorCode::InvalidInput, "bemt_T_N invalid");
        LIFT_BEMT_REQUIRE(is_finite(bemt_P_W) && bemt_P_W >= 0.0, ErrorCode::InvalidInput, "bemt_P_W invalid");

        LIFT_BEMT_REQUIRE(is_finite(correction_thrust) && correction_thrust > 0.0 && correction_thrust < 10.0, ErrorCode::InvalidInput, "correction_thrust invalid");
        LIFT_BEMT_REQUIRE(is_finite(correction_power) && correction_power > 0.0 && correction_power < 10.0, ErrorCode::InvalidInput, "correction_power invalid");
    }
};

struct CfdManifest final {
    std::string manifest_id;           // e.g., "run_2026_01_03_001"
    std::string created_utc_iso8601;   // provided by caller (no time libs here)
    std::string notes;

    std::vector<CfdJob> jobs;

    void validate() const {
        LIFT_BEMT_REQUIRE(!manifest_id.empty(), ErrorCode::InvalidInput, "manifest_id empty");
        for (const auto& j : jobs) j.validate();
    }
};

// Selection policy for promoting BEMT candidates to CFD.
struct CfdSelectionPolicy final {
    std::size_t top_n = 25;

    // Only promote GO cases by default.
    bool require_go = true;

    // Sort key (cheap and stable).
    // If true: ascending hover power.
    // else: descending hover thrust.
    bool sort_by_lowest_hover_power = true;

    // Tier assignment
    CfdTier tier = CfdTier::CFD0_ActuatorDisk;

    void validate() const {
        LIFT_BEMT_REQUIRE(top_n >= 1 && top_n <= 1000000, ErrorCode::InvalidInput, "top_n invalid");
    }
};

// Build jobs from closeout rows + GO/NO-GO reports.
// geometry_ref_prefix is prepended to each case_id (simple convention).
CfdManifest build_cfd_manifest(const std::string& manifest_id,
                               const std::string& created_utc_iso8601,
                               const std::string& notes,
                               const std::vector<CloseoutRow>& closeout_rows,
                               const std::vector<GoNoGoReport>& gonogo_reports,
                               const CfdSelectionPolicy& policy,
                               const std::string& geometry_ref_prefix = "exports/");

// JSON/CSV serialization
std::string cfd_manifest_json(const CfdManifest& m);
std::string cfd_manifest_csv(const CfdManifest& m);

} // namespace lift::bemt
