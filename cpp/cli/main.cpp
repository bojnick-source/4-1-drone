/*
================================================================================
Fragment 1.0 — CLI: Main Entry Point (lift_cli)
FILE: cpp/cli/main.cpp

Purpose:
  - Primary command-line interface for the lift optimization engine.
  - Provides access to key operations:
    * Design validation
    * Physics evaluation (hover power, disk area)
    * Cache key generation
    * Design hashing
    * Basic sanity checks

Usage:
  lift_cli [command] [options]

Commands:
  validate   - Validate a design configuration
  hash       - Generate hash for a design
  disk-area  - Compute effective disk area
  hover      - Compute hover power
  help       - Show help message

Hardening:
  - Explicit error codes for CI integration
  - No silent failures
  - Deterministic output format
================================================================================
*/

#include "engine/core/design.hpp"
#include "engine/core/errors.hpp"
#include "engine/core/settings.hpp"
#include "engine/core/logging.hpp"
#include "engine/core/design_hash.hpp"
#include "engine/physics/disk_area.hpp"
#include "engine/physics/hover_momentum.hpp"

#include <iostream>
#include <string>
#include <cstdlib>
#include <cmath>

using namespace lift;

// Exit codes for CI integration
enum ExitCode {
  SUCCESS = 0,
  INVALID_ARGS = 1,
  VALIDATION_FAILED = 2,
  COMPUTATION_FAILED = 3,
  IO_ERROR = 4
};

void print_help() {
  std::cout << R"(
lift_cli - DARPA LIFT Design Optimization Engine

Usage:
  lift_cli [command] [options]

Commands:
  validate      Validate a design configuration (demo)
  hash          Generate hash for a design (demo)
  disk-area     Compute effective disk area (demo)
  hover         Compute hover power (demo)
  help          Show this help message

Examples:
  lift_cli validate
  lift_cli hash
  lift_cli disk-area
  lift_cli hover

Note:
  This is a minimal CLI harness. Full optimizer integration
  and config file loading will be added in later fragments.

Exit Codes:
  0 - Success
  1 - Invalid arguments
  2 - Validation failed
  3 - Computation failed
  4 - I/O error
)";
}

// Create a demo design for testing
Design create_demo_design() {
  Design d;
  d.name = "Demo Quad";
  d.arch = Architecture::Multicopter_Open;
  d.rotor_count = 4;
  d.rotor_radius_m = 0.30;  // 30cm radius
  d.rotor_solidity = 0.05;
  d.rotor_rpm = 5000.0;
  d.rotor_tip_speed_mps = d.rotor_radius_m * (d.rotor_rpm * 2.0 * M_PI / 60.0);
  
  // Mass model
  d.mass.structural_kg = 5.0;
  d.mass.propulsion_kg = 8.0;
  d.mass.energy_kg = 10.0;
  d.mass.avionics_kg = 1.5;
  d.mass.payload_interface_kg = 0.5;
  d.mass.misc_kg = 0.0;
  
  // Aero model
  d.aero.CdS_m2 = 0.15;
  d.aero.lift_to_drag = 0.0;
  
  // Power system
  d.power.rotor_max_shaft_W = 10000.0;
  d.power.rotor_cont_shaft_W = 8000.0;
  d.power.bus_voltage_V = 48.0;
  
  return d;
}

int cmd_validate() {
  std::cout << "=== Design Validation Demo ===\n";
  
  try {
    Design d = create_demo_design();
    d.validate_or_throw();
    
    std::cout << "Design: " << d.name << "\n";
    std::cout << "Architecture: Multicopter_Open\n";
    std::cout << "Rotor count: " << d.rotor_count << "\n";
    std::cout << "Rotor radius: " << d.rotor_radius_m << " m\n";
    std::cout << "Total mass: " << d.aircraft_mass_kg() << " kg\n";
    std::cout << "\nValidation: PASSED\n";
    
    return ExitCode::SUCCESS;
    
  } catch (const ValidationError& e) {
    std::cerr << "Validation FAILED: " << e.what() << "\n";
    return ExitCode::VALIDATION_FAILED;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return ExitCode::COMPUTATION_FAILED;
  }
}

int cmd_hash() {
  std::cout << "=== Design Hash Demo ===\n";
  
  try {
    Design d = create_demo_design();
    d.validate_or_throw();
    
    std::string hash = hash_design_schema_hex(d);
    
    std::cout << "Design: " << d.name << "\n";
    std::cout << "Hash: " << hash << "\n";
    std::cout << "\nHash computation: SUCCESS\n";
    
    return ExitCode::SUCCESS;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return ExitCode::COMPUTATION_FAILED;
  }
}

int cmd_disk_area() {
  std::cout << "=== Disk Area Computation Demo ===\n";
  
  try {
    Design d = create_demo_design();
    d.validate_or_throw();
    
    DiskAreaResult result = compute_effective_disk_area(d);
    
    std::cout << "Design: " << d.name << "\n";
    std::cout << "Rotor count: " << d.rotor_count << "\n";
    std::cout << "Rotor radius: " << d.rotor_radius_m << " m\n";
    std::cout << "\nResults:\n";
    std::cout << "  Single disk area: " << result.A_single_m2 << " m²\n";
    std::cout << "  Total disk area: " << result.A_total_m2 << " m²\n";
    std::cout << "  Effective disk count: " << result.effective_disk_count << "\n";
    std::cout << "  Notes: " << result.notes << "\n";
    std::cout << "\nComputation: SUCCESS\n";
    
    return ExitCode::SUCCESS;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return ExitCode::COMPUTATION_FAILED;
  }
}

int cmd_hover() {
  std::cout << "=== Hover Power Computation Demo ===\n";
  
  try {
    Design d = create_demo_design();
    d.validate_or_throw();
    
    DiskAreaResult disk = compute_effective_disk_area(d);
    
    EvalSettings settings = EvalSettings::defaults();
    
    // Compute thrust needed for hover (1g)
    double thrust_N = d.aircraft_mass_kg() * 9.81;
    
    HoverMomentumResult hover = hover_momentum_power(
      thrust_N,
      disk.A_total_m2,
      settings
    );
    
    std::cout << "Design: " << d.name << "\n";
    std::cout << "Aircraft mass: " << d.aircraft_mass_kg() << " kg\n";
    std::cout << "Thrust required: " << thrust_N << " N\n";
    std::cout << "Total disk area: " << disk.A_total_m2 << " m²\n";
    std::cout << "\nResults:\n";
    std::cout << "  Disk loading: " << hover.disk_loading_N_per_m2 << " N/m²\n";
    std::cout << "  P_induced (ideal): " << hover.P_induced_ideal_W << " W\n";
    std::cout << "  P_induced (losses): " << hover.P_induced_W << " W\n";
    std::cout << "  P_total (with FM): " << hover.P_total_W << " W\n";
    std::cout << "  FM used: " << hover.FM_used << "\n";
    std::cout << "  Density: " << hover.rho_used << " kg/m³\n";
    std::cout << "\nComputation: SUCCESS\n";
    
    return ExitCode::SUCCESS;
    
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return ExitCode::COMPUTATION_FAILED;
  }
}

int main(int argc, char** argv) {
  // Parse command
  std::string cmd = (argc >= 2) ? std::string(argv[1]) : "help";
  
  if (cmd == "help" || cmd == "-h" || cmd == "--help") {
    print_help();
    return ExitCode::SUCCESS;
  }
  
  if (cmd == "validate") {
    return cmd_validate();
  }
  
  if (cmd == "hash") {
    return cmd_hash();
  }
  
  if (cmd == "disk-area") {
    return cmd_disk_area();
  }
  
  if (cmd == "hover") {
    return cmd_hover();
  }
  
  std::cerr << "Unknown command: " << cmd << "\n";
  std::cerr << "Run 'lift_cli help' for usage information.\n";
  return ExitCode::INVALID_ARGS;
}
