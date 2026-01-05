#pragma once
/*
================================================================================
Fragment 1.19 — Optimization: Candidate Generator (Design Space Sampling)
FILE: cpp/engine/optimization/candidate_generator.hpp

Purpose:
  - Generate candidate designs for evaluation
  - Support multiple sampling strategies:
    * Random sampling (Monte Carlo)
    * Latin Hypercube Sampling (LHS)
    * Grid sampling
    * Sobol sequences
  - Apply design space bounds and constraints
  - Ensure deterministic generation (seeded RNG)

Use cases:
  - Initial population for GA/PSO
  - Design space exploration
  - Sensitivity analysis
  - Baseline search

Hardening:
  - Explicit bounds checking
  - Deterministic seeding for reproducibility
  - Validation of generated candidates
  - Constraint pre-filtering

Design space parameters:
  - Rotor count: discrete {4, 6, 8}
  - Rotor radius: continuous [0.15, 0.50] m
  - Rotor solidity: continuous [0.03, 0.15]
  - Rotor RPM: continuous [3000, 8000]
  - Mass components: continuous ranges
  - Architecture: discrete enum
================================================================================
*/

#include "engine/core/design.hpp"
#include "engine/core/settings.hpp"

#include <cstdint>
#include <vector>

namespace lift {

// Design space bounds for optimization
struct DesignSpaceBounds {
  // Rotor geometry
  std::vector<int> rotor_count_options = {4, 6, 8};
  double rotor_radius_min_m = 0.15;
  double rotor_radius_max_m = 0.50;
  double rotor_solidity_min = 0.03;
  double rotor_solidity_max = 0.15;
  double rotor_rpm_min = 3000.0;
  double rotor_rpm_max = 8000.0;
  
  // Mass bounds
  double structural_kg_min = 2.0;
  double structural_kg_max = 10.0;
  double propulsion_kg_min = 4.0;
  double propulsion_kg_max = 15.0;
  double energy_kg_min = 5.0;
  double energy_kg_max = 20.0;
  double avionics_kg_min = 0.5;
  double avionics_kg_max = 3.0;
  
  // Aero bounds
  double CdS_min_m2 = 0.05;
  double CdS_max_m2 = 0.50;
  
  // Validation
  void validate_or_throw() const;
};

// Sampling strategy
enum class SamplingStrategy {
  Random = 0,          // Uniform random sampling
  LatinHypercube = 1,  // LHS for better space coverage
  Grid = 2,            // Regular grid
  Sobol = 3            // Sobol sequence (quasi-random)
};

// Candidate generation options
struct CandidateGenOptions {
  SamplingStrategy strategy = SamplingStrategy::Random;
  uint64_t seed = 1;              // For deterministic generation
  int count = 100;                // Number of candidates to generate
  bool validate_candidates = true; // Run validate_or_throw on each
  bool apply_constraints = true;   // Pre-filter infeasible designs
};

// Generate candidate designs
std::vector<Design> generate_candidates(const DesignSpaceBounds& bounds,
                                        const CandidateGenOptions& options);

// Generate a single random candidate
Design generate_random_candidate(const DesignSpaceBounds& bounds,
                                 uint64_t seed);

// Check if design satisfies basic constraints (pre-filter)
bool satisfies_basic_constraints(const Design& d);

}  // namespace lift
