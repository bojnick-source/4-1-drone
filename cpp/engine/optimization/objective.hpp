#pragma once
/*
================================================================================
Fragment 1.20 — Optimization: Objective Function (Scoring & Penalties)
FILE: cpp/engine/optimization/objective.hpp

Purpose:
  - Define objective function for design optimization
  - Combine multiple metrics into single score
  - Apply penalties for constraint violations
  - Support multi-objective optimization setup

Primary objective:
  - Maximize payload ratio (payload mass / aircraft mass)
  - Subject to mission feasibility constraints

Secondary objectives (can be weighted):
  - Minimize mission time
  - Minimize energy consumption
  - Maximize control authority margins
  - Minimize structural complexity

Hardening:
  - Explicit penalty coefficients
  - Traceable score breakdown
  - Deterministic computation
  - NaN-safe aggregation

Score components:
  1. Base fitness (payload ratio, mission performance)
  2. Constraint penalties (mass limit, disk loading, tip speed)
  3. Preference bonuses (margin cushions, robustness)
  4. Infeasibility flag (hard failure → -infinity)
================================================================================
*/

#include "engine/core/design.hpp"
#include "engine/core/mission_spec.hpp"
#include "engine/core/settings.hpp"
#include "engine/physics/disk_area.hpp"
#include "engine/physics/hover_momentum.hpp"

#include <string>
#include <vector>

namespace lift {

// Objective evaluation result
struct ObjectiveResult {
  double score = 0.0;           // Overall score (higher is better)
  double base_fitness = 0.0;    // Base objective value
  double penalty_total = 0.0;   // Sum of penalties (negative)
  bool is_feasible = true;      // Hard constraint satisfaction
  
  // Breakdown for debugging
  double payload_ratio = 0.0;
  double mass_penalty = 0.0;
  double disk_loading_penalty = 0.0;
  double tip_speed_penalty = 0.0;
  double power_penalty = 0.0;
  
  std::string notes;
};

// Objective function options
struct ObjectiveOptions {
  // Penalty coefficients
  double mass_penalty_coeff = 1000.0;        // Penalty per kg over limit
  double disk_loading_penalty_coeff = 10.0;  // Penalty for extreme DL
  double tip_speed_penalty_coeff = 100.0;    // Penalty for Mach violations
  double power_penalty_coeff = 1.0;          // Penalty for insufficient power
  
  // Feasibility gates
  double max_aircraft_mass_kg = 30.0;
  double min_payload_ratio = 3.0;
  double max_tip_mach = 0.70;
  double max_disk_loading_N_m2 = 600.0;
  double min_disk_loading_N_m2 = 100.0;
};

// Evaluate objective function for a design
ObjectiveResult evaluate_objective(const Design& d,
                                    const MissionSpec& mission,
                                    const EvalSettings& settings,
                                    const ObjectiveOptions& obj_opt);

// Convenience: evaluate and return only score
double evaluate_score(const Design& d,
                      const MissionSpec& mission,
                      const EvalSettings& settings,
                      const ObjectiveOptions& obj_opt);

}  // namespace lift
