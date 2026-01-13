#pragma once
/*
Fragment 1.18 — Physics: CFD Results Container (Post-Processing Integration)
FILE: cpp/engine/physics/cfd_results.hpp

Purpose:
  - Define container for CFD simulation results
  - Provide integration point for external CFD solvers (OpenFOAM, SU2, etc.)
  - Store computed forces, moments, power, and flow field statistics
  - Enable validation of lower-fidelity models against CFD

Use cases:
  - Import CFD results for closeout validation
  - Calibrate momentum theory / BEMT models
  - Document verification artifacts
  - Store uncertainty bounds from CFD mesh/solver studies

Hardening:
  - Explicit metadata (solver, version, mesh stats, convergence)
  - Traceable provenance (input hash, timestamp, runtime)
  - NaN-safe fields for optional data
  - Validation hooks for sanity checks

Fields:
  - Forces & moments (body frame)
  - Rotor performance (thrust, torque, power per rotor)
  - Drag breakdown (parasite, induced, profile)
  - Flow statistics (pressure, velocity, vorticity)
  - Convergence metrics
*/

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace lift {

// Sentinel for unset values
inline constexpr double kCfdUnset = std::numeric_limits<double>::quiet_NaN();

// CFD solver metadata
struct CfdSolverInfo {
  std::string solver_name;       // e.g., "OpenFOAM", "SU2", "CFX"
  std::string solver_version;    // e.g., "v2112", "7.5.1"
  std::string turbulence_model;  // e.g., "k-omega SST", "SA", "laminar"
  std::string mesh_id;           // mesh identifier or hash
  int cell_count = 0;
  int boundary_layer_cells = 0;
  double y_plus_mean = kCfdUnset;
  double y_plus_max = kCfdUnset;
};

// Convergence info
struct CfdConvergence {
  int iterations = 0;
  double residual_continuity = kCfdUnset;
  double residual_momentum = kCfdUnset;
  double residual_energy = kCfdUnset;
  bool converged = false;
  double runtime_s = kCfdUnset;
  std::string notes;
};

// Single rotor CFD outputs
struct RotorCfdResult {
  int rotor_index = -1;
  double thrust_N = kCfdUnset;
  double torque_Nm = kCfdUnset;
  double power_W = kCfdUnset;
  double CT = kCfdUnset;  // thrust coefficient
  double CP = kCfdUnset;  // power coefficient
  double FM = kCfdUnset;  // figure of merit (if computed)
  std::string notes;
};

// Total forces and moments (body frame)
struct CfdForceMoment {
  double Fx_N = kCfdUnset;  // forward
  double Fy_N = kCfdUnset;  // right
  double Fz_N = kCfdUnset;  // down
  double Mx_Nm = kCfdUnset; // roll moment
  double My_Nm = kCfdUnset; // pitch moment
  double Mz_Nm = kCfdUnset; // yaw moment
};

// Drag breakdown
struct CfdDragBreakdown {
  double D_total_N = kCfdUnset;
  double D_parasite_N = kCfdUnset;
  double D_induced_N = kCfdUnset;
  double D_profile_N = kCfdUnset;
  double CdS_m2 = kCfdUnset;  // equivalent drag area
  std::string notes;
};

// Flow statistics (optional, for validation)
struct CfdFlowStats {
  double V_mean_mps = kCfdUnset;
  double P_mean_Pa = kCfdUnset;
  double rho_mean_kg_m3 = kCfdUnset;
  double T_mean_K = kCfdUnset;
  double vorticity_max_1_s = kCfdUnset;
  std::string notes;
};

// Main CFD results container
struct CfdResults {
  // Provenance
  std::string design_hash;      // links to Design
  std::string input_hash;       // hash of CFD input files
  std::string timestamp;        // ISO 8601
  
  // Solver info
  CfdSolverInfo solver;
  CfdConvergence convergence;
  
  // Operating condition
  double freestream_V_mps = kCfdUnset;
  double freestream_rho_kg_m3 = kCfdUnset;
  double freestream_P_Pa = kCfdUnset;
  double freestream_T_K = kCfdUnset;
  double altitude_m = kCfdUnset;
  
  // Results
  CfdForceMoment total_force_moment;
  std::vector<RotorCfdResult> rotors;
  CfdDragBreakdown drag;
  CfdFlowStats flow_stats;
  
  // Optional: uncertainty bounds (from mesh/solver studies)
  double uncertainty_thrust_pct = kCfdUnset;
  double uncertainty_power_pct = kCfdUnset;
  double uncertainty_drag_pct = kCfdUnset;
  
  // Validation
  void validate_or_warn() const;
};

}  // namespace lift
// ============================================================================
// CFD Results Data Structures
// Fragment 1.4.24 — CFD Results Data Containers (C++)
// File: cpp/engine/physics/cfd_results.hpp
// ============================================================================
//
// Purpose:
// - Define data containers for CFD simulation results.
// - Header-only data structures for now (implementation minimal).
// - Provide stable, header-only data structures for CFD result ingestion.
// - Designed to hold rotor performance maps, flow-field snapshots, or
//   BEMT-to-CFD comparison metrics.
//
// ============================================================================

#include <string>
#include <vector>

namespace lift::cfd {

// Placeholder CFD result structures
// These would be populated by CFD analysis tools

struct CfdCase {
    std::string case_id;
    double mach = 0.0;
    double reynolds = 0.0;
    double alpha_deg = 0.0;
    double cl = 0.0;
    double cd = 0.0;
    double cm = 0.0;
};

struct CfdResults {
    std::string geometry_id;
    std::vector<CfdCase> cases;
// Simple container for CFD result metadata
struct CfdResultMeta {
    std::string case_id;
    std::string solver_version;
    double timestamp = 0.0;
    std::string notes;
};

// Placeholder for CFD rotor performance data
struct CfdRotorPerformance {
    double thrust_N = 0.0;
    double torque_Nm = 0.0;
    double power_W = 0.0;
    double rpm = 0.0;
    double pitch_deg = 0.0;
};

// Container for a CFD result case
struct CfdResult {
    CfdResultMeta meta;
    std::vector<CfdRotorPerformance> rotor_data;
    
    void validate() const {
        // Placeholder for future validation
    }
};

} // namespace lift::cfd
