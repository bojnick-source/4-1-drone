/*
================================================================================
Fragment 4.2 — CLI: Optimizer Demo (Candidate Generation & Scoring)
FILE: cpp/cli/optimizer_demo.cpp

Purpose:
  - Demonstrate candidate generation and objective evaluation
  - Test computational engine components
  - Generate batch CSV output

Usage:
  optimizer_demo [num_candidates] [output_csv]
Examples:
  optimizer_demo 10 candidates.csv
================================================================================
*/

#include "engine/core/design.hpp"
#include "engine/core/mission_spec.hpp"
#include "engine/core/settings.hpp"
#include "engine/optimization/candidate_generator.hpp"
#include "engine/optimization/objective.hpp"
#include "engine/exports/stats_report_csv.hpp"
#include "engine/analysis/closeout_types.hpp"

#include <iostream>
#include <string>
#include <vector>

static std::string default_csv_path() {
  return "candidates.csv";
}

int main(int argc, char** argv) {
  int num_candidates = (argc >= 2) ? std::atoi(argv[1]) : 10;
  std::string csv_path = (argc >= 3) ? std::string(argv[2]) : default_csv_path();
  
  std::cout << "=== Optimizer Demo: Candidate Generation & Scoring ===\n";
  std::cout << "Generating " << num_candidates << " candidates...\n";
  
  // Setup
  lift::DesignSpaceBounds bounds;
  lift::CandidateGenOptions gen_opt;
  gen_opt.count = num_candidates;
  gen_opt.seed = 42;
  gen_opt.strategy = lift::SamplingStrategy::Random;
  
  lift::MissionSpec mission = lift::MissionSpec::darpa_lift_default();
  lift::EvalSettings settings = lift::EvalSettings::defaults();
  lift::ObjectiveOptions obj_opt;
  
  // Generate candidates
  std::vector<lift::Design> candidates = lift::generate_candidates(bounds, gen_opt);
  
  std::cout << "Generated " << candidates.size() << " valid candidates\n";
  
  // Evaluate and create closeout reports
  std::vector<lift::CloseoutReport> reports;
  int feasible_count = 0;
  
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto& d = candidates[i];
    
    // Evaluate objective
    lift::ObjectiveResult obj = lift::evaluate_objective(d, mission, settings, obj_opt);
    
    if (obj.is_feasible) {
      feasible_count++;
    }
    
    // Create minimal closeout report
    lift::CloseoutReport r;
    r.variant_name = d.name;
    r.geom_hash = std::to_string(i);
    
    r.mass_delta.baseline_aircraft_mass_kg = 25.0;
    r.mass_delta.resulting_aircraft_mass_kg = d.aircraft_mass_kg();
    r.mass_delta.delta_mass_total_kg = d.aircraft_mass_kg() - 25.0;
    r.mass_delta.resulting_payload_ratio = obj.payload_ratio;
    
    r.gate_result.decision = obj.is_feasible ? lift::GateDecision::Go : lift::GateDecision::NoGo;
    
    reports.push_back(r);
    
    if (i < 3) {  // Print first 3
      std::cout << "\nCandidate " << i << ": " << d.name << "\n";
      std::cout << "  Rotor count: " << d.rotor_count << "\n";
      std::cout << "  Rotor radius: " << d.rotor_radius_m << " m\n";
      std::cout << "  Aircraft mass: " << d.aircraft_mass_kg() << " kg\n";
      std::cout << "  Payload ratio: " << obj.payload_ratio << "\n";
      std::cout << "  Score: " << obj.score << "\n";
      std::cout << "  Feasible: " << (obj.is_feasible ? "Yes" : "No") << "\n";
    }
  }
  
  std::cout << "\n=== Summary ===\n";
  std::cout << "Total candidates: " << candidates.size() << "\n";
  std::cout << "Feasible: " << feasible_count << "\n";
  std::cout << "Infeasible: " << (candidates.size() - feasible_count) << "\n";
  
  // Export to CSV
  lift::CsvExportOptions csv_opt;
  bool success = lift::write_closeout_csv_file(reports, csv_path, csv_opt);
  
  if (success) {
    std::cout << "\nWrote CSV: " << csv_path << "\n";
    return 0;
  } else {
    std::cerr << "ERROR: Failed to write CSV\n";
    return 10;
  }
}
