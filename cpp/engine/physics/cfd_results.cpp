/*
Fragment 1.18 — Physics: CFD Results Container Implementation
FILE: cpp/engine/physics/cfd_results.cpp
*/

#include "cfd_results.hpp"
#include "engine/core/logging.hpp"

#include <cmath>

namespace lift {

void CfdResults::validate_or_warn() const {
  // Check basic sanity
  if (solver.solver_name.empty()) {
    log(LogLevel::WARN, "CfdResults: solver_name is empty");
  }
  
  if (convergence.iterations <= 0) {
    log(LogLevel::WARN, "CfdResults: iterations <= 0, possibly not converged");
  }
  
  if (!convergence.converged) {
    log(LogLevel::WARN, "CfdResults: convergence flag is false");
  }
  
  // Check operating condition
  if (std::isnan(freestream_rho_kg_m3) || freestream_rho_kg_m3 <= 0.0) {
    log(LogLevel::WARN, "CfdResults: freestream density unset or invalid");
  }
  
  // Check forces
  if (std::isnan(total_force_moment.Fz_N)) {
    log(LogLevel::WARN, "CfdResults: vertical force (Fz_N) is unset");
  }
  
  // Check rotors
  if (rotors.empty()) {
    log(LogLevel::WARN, "CfdResults: no rotor results provided");
  } else {
    for (const auto& rotor : rotors) {
      if (std::isnan(rotor.thrust_N)) {
        log(LogLevel::WARN, "CfdResults: rotor " + std::to_string(rotor.rotor_index) + " thrust is unset");
      }
      if (std::isnan(rotor.power_W)) {
        log(LogLevel::WARN, "CfdResults: rotor " + std::to_string(rotor.rotor_index) + " power is unset");
      }
    }
  }
  
  // Check drag
  if (std::isnan(drag.D_total_N) && std::isnan(drag.CdS_m2)) {
    log(LogLevel::WARN, "CfdResults: both D_total and CdS are unset");
  }
}

}  // namespace lift
// ============================================================================
// Fragment 1.4.25 — CFD Results Implementation (C++)
// File: cpp/engine/physics/cfd_results.cpp
// ============================================================================
//
// This translation unit exists to satisfy build wiring and provide a stable
// place for future non-inline helpers (e.g., case lookup, result merging,
// and BEMT↔CFD comparison utilities).
//

#include "engine/physics/cfd_results.hpp"

namespace lift::cfd {
// Intentionally minimal (data containers are header-only for now).
} // namespace lift::cfd
