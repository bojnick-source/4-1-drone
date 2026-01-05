/*
================================================================================
Fragment 1.19 — Optimization: Candidate Generator Implementation
FILE: cpp/engine/optimization/candidate_generator.cpp
================================================================================
*/

#include "candidate_generator.hpp"
#include "engine/core/errors.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace lift {

void DesignSpaceBounds::validate_or_throw() const {
  if (rotor_count_options.empty()) {
    throw ValidationError("DesignSpaceBounds: rotor_count_options is empty");
  }
  if (rotor_radius_min_m <= 0.0 || rotor_radius_max_m <= rotor_radius_min_m) {
    throw ValidationError("DesignSpaceBounds: invalid rotor radius bounds");
  }
  if (rotor_solidity_min <= 0.0 || rotor_solidity_max <= rotor_solidity_min) {
    throw ValidationError("DesignSpaceBounds: invalid rotor solidity bounds");
  }
  if (rotor_rpm_min <= 0.0 || rotor_rpm_max <= rotor_rpm_min) {
    throw ValidationError("DesignSpaceBounds: invalid rotor RPM bounds");
  }
}

bool satisfies_basic_constraints(const Design& d) {
  // Check basic physics constraints
  
  // Total mass must be reasonable
  double total_mass = d.aircraft_mass_kg();
  if (total_mass < 10.0 || total_mass > 50.0) return false;
  
  // Rotor tip speed limit (Mach limit)
  double tip_speed = d.rotor_tip_speed_mps;
  if (tip_speed <= 0.0) {
    tip_speed = d.rotor_radius_m * (d.rotor_rpm * 2.0 * M_PI / 60.0);
  }
  if (tip_speed > 220.0) return false;  // ~Mach 0.65 at sea level
  
  // Disk loading sanity (rough check)
  double A_single = M_PI * d.rotor_radius_m * d.rotor_radius_m;
  double A_total = A_single * d.rotor_count;
  if (d.is_coaxial) {
    A_total = A_single * d.coax_pairs;  // Coax shares footprint
  }
  double disk_loading = (total_mass * 9.81) / A_total;
  if (disk_loading < 50.0 || disk_loading > 800.0) return false;
  
  return true;
}

Design generate_random_candidate(const DesignSpaceBounds& bounds,
                                 uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  
  Design d;
  d.name = "candidate_" + std::to_string(seed);
  
  // Architecture (for now, just Multicopter_Open)
  d.arch = Architecture::Multicopter_Open;
  d.is_coaxial = false;
  d.has_shroud = false;
  
  // Rotor count (discrete choice)
  std::uniform_int_distribution<size_t> rotor_idx(0, bounds.rotor_count_options.size() - 1);
  d.rotor_count = bounds.rotor_count_options[rotor_idx(rng)];
  
  // Rotor geometry (continuous)
  d.rotor_radius_m = bounds.rotor_radius_min_m + 
                     unit(rng) * (bounds.rotor_radius_max_m - bounds.rotor_radius_min_m);
  
  d.rotor_solidity = bounds.rotor_solidity_min + 
                     unit(rng) * (bounds.rotor_solidity_max - bounds.rotor_solidity_min);
  
  d.rotor_rpm = bounds.rotor_rpm_min + 
                unit(rng) * (bounds.rotor_rpm_max - bounds.rotor_rpm_min);
  
  // Compute tip speed
  d.rotor_tip_speed_mps = d.rotor_radius_m * (d.rotor_rpm * 2.0 * M_PI / 60.0);
  
  // Mass model (continuous)
  d.mass.structural_kg = bounds.structural_kg_min + 
                         unit(rng) * (bounds.structural_kg_max - bounds.structural_kg_min);
  
  d.mass.propulsion_kg = bounds.propulsion_kg_min + 
                         unit(rng) * (bounds.propulsion_kg_max - bounds.propulsion_kg_min);
  
  d.mass.energy_kg = bounds.energy_kg_min + 
                     unit(rng) * (bounds.energy_kg_max - bounds.energy_kg_min);
  
  d.mass.avionics_kg = bounds.avionics_kg_min + 
                       unit(rng) * (bounds.avionics_kg_max - bounds.avionics_kg_min);
  
  d.mass.payload_interface_kg = 0.3 + unit(rng) * 1.0;
  d.mass.misc_kg = 0.1 + unit(rng) * 0.5;
  
  // Aero model
  d.aero.CdS_m2 = bounds.CdS_min_m2 + 
                  unit(rng) * (bounds.CdS_max_m2 - bounds.CdS_min_m2);
  d.aero.lift_to_drag = 0.0;
  
  // Power system (simple defaults for now)
  d.power.rotor_max_shaft_W = 15000.0;
  d.power.rotor_cont_shaft_W = 12000.0;
  d.power.bus_voltage_V = 48.0;
  
  return d;
}

std::vector<Design> generate_candidates(const DesignSpaceBounds& bounds,
                                        const CandidateGenOptions& options) {
  bounds.validate_or_throw();
  
  std::vector<Design> candidates;
  candidates.reserve(options.count);
  
  uint64_t seed = options.seed;
  
  for (int i = 0; i < options.count; ++i) {
    Design d = generate_random_candidate(bounds, seed + static_cast<uint64_t>(i));
    
    // Apply constraints filter
    if (options.apply_constraints) {
      if (!satisfies_basic_constraints(d)) {
        // Skip infeasible candidate, try again with new seed
        continue;
      }
    }
    
    // Validate
    if (options.validate_candidates) {
      try {
        d.validate_or_throw();
      } catch (const ValidationError&) {
        // Skip invalid candidate
        continue;
      }
    }
    
    candidates.push_back(d);
  }
  
  return candidates;
}

}  // namespace lift
