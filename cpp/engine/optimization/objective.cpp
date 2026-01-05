/*
================================================================================
Fragment 1.20 — Optimization: Objective Function Implementation
FILE: cpp/engine/optimization/objective.cpp
================================================================================
*/

#include "objective.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lift {

ObjectiveResult evaluate_objective(const Design& d,
                                    const MissionSpec& mission,
                                    const EvalSettings& settings,
                                    const ObjectiveOptions& obj_opt) {
  ObjectiveResult res;
  res.is_feasible = true;
  
  // Compute basic metrics
  double aircraft_mass_kg = d.aircraft_mass_kg();
  double payload_kg = mission.min_payload_mass_kg;
  double payload_ratio = payload_kg / aircraft_mass_kg;
  
  res.payload_ratio = payload_ratio;
  
  // Base fitness: payload ratio (higher is better)
  res.base_fitness = payload_ratio;
  
  // Constraint 1: Aircraft mass limit
  if (aircraft_mass_kg > obj_opt.max_aircraft_mass_kg) {
    double excess = aircraft_mass_kg - obj_opt.max_aircraft_mass_kg;
    res.mass_penalty = -obj_opt.mass_penalty_coeff * excess;
    res.is_feasible = false;
  }
  
  // Constraint 2: Minimum payload ratio
  if (payload_ratio < obj_opt.min_payload_ratio) {
    double deficit = obj_opt.min_payload_ratio - payload_ratio;
    res.mass_penalty += -obj_opt.mass_penalty_coeff * deficit * 10.0;  // Scale up for payload ratio
    res.is_feasible = false;
  }
  
  // Constraint 3: Tip speed (Mach limit)
  double tip_speed = d.rotor_tip_speed_mps;
  if (tip_speed <= 0.0) {
    tip_speed = d.rotor_radius_m * (d.rotor_rpm * 2.0 * M_PI / 60.0);
  }
  double speed_of_sound = 340.0;  // m/s at sea level
  double tip_mach = tip_speed / speed_of_sound;
  
  if (tip_mach > obj_opt.max_tip_mach) {
    double excess_mach = tip_mach - obj_opt.max_tip_mach;
    res.tip_speed_penalty = -obj_opt.tip_speed_penalty_coeff * excess_mach;
    res.is_feasible = false;
  }
  
  // Constraint 4: Disk loading bounds
  DiskAreaResult disk = compute_effective_disk_area(d);
  double thrust_N = aircraft_mass_kg * 9.81;
  double disk_loading = thrust_N / disk.A_total_m2;
  
  if (disk_loading > obj_opt.max_disk_loading_N_m2) {
    double excess = disk_loading - obj_opt.max_disk_loading_N_m2;
    res.disk_loading_penalty += -obj_opt.disk_loading_penalty_coeff * (excess / 100.0);
  }
  
  if (disk_loading < obj_opt.min_disk_loading_N_m2) {
    double deficit = obj_opt.min_disk_loading_N_m2 - disk_loading;
    res.disk_loading_penalty += -obj_opt.disk_loading_penalty_coeff * (deficit / 100.0);
  }
  
  // Constraint 5: Power feasibility (simple check)
  HoverMomentumResult hover = hover_momentum_power(thrust_N, disk.A_total_m2, settings);
  double power_required_per_rotor = hover.P_total_W / static_cast<double>(d.rotor_count);
  
  if (power_required_per_rotor > d.power.rotor_max_shaft_W) {
    double deficit = power_required_per_rotor - d.power.rotor_max_shaft_W;
    res.power_penalty = -obj_opt.power_penalty_coeff * (deficit / 1000.0);
    res.is_feasible = false;
  }
  
  // Total penalty
  res.penalty_total = res.mass_penalty + res.disk_loading_penalty + 
                      res.tip_speed_penalty + res.power_penalty;
  
  // Final score
  if (res.is_feasible) {
    res.score = res.base_fitness + res.penalty_total;
  } else {
    // Infeasible designs get large negative score
    res.score = -1000.0 + res.penalty_total;
  }
  
  return res;
}

double evaluate_score(const Design& d,
                      const MissionSpec& mission,
                      const EvalSettings& settings,
                      const ObjectiveOptions& obj_opt) {
  return evaluate_objective(d, mission, settings, obj_opt).score;
}

}  // namespace lift
