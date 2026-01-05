#include "engine/core/design.hpp"
#include "engine/physics/disk_area.hpp"

#include <exception>
#include <iostream>

// Minimal harness for the lift_engine library.
// Provides a simple sanity path so the CMake target builds and can be used for
// quick smoke tests of the disk-area computation.
int main() {
  try {
    lift::Design d{};
    d.name = "lift_cli_demo";
    d.rotor_count = 4;
    d.rotor_radius_m = 0.65;
    d.rotor_solidity = 0.08;
    d.rotor_rpm = 1200.0;

    // Basic mass model so validation passes.
    d.mass.structural_kg = 5.0;
    d.mass.propulsion_kg = 4.0;
    d.mass.energy_kg = 6.0;
    d.mass.avionics_kg = 1.0;
    d.mass.payload_interface_kg = 1.0;
    d.mass.misc_kg = 0.5;

    const auto res = lift::compute_effective_disk_area(d);
    std::cout << "Design: " << d.name << "\n";
    std::cout << "Rotor count: " << d.rotor_count << "\n";
    std::cout << "Disk area (single): " << res.A_single_m2 << " m^2\n";
    std::cout << "Disk area (total):  " << res.A_total_m2 << " m^2\n";
    std::cout << res.notes << "\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "lift_cli error: " << e.what() << "\n";
  }
  return 1;
}
