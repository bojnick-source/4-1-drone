#pragma once
// ============================================================================
// CFD Results Data Structures
// File: cpp/engine/physics/cfd_results.hpp
// ============================================================================
//
// Purpose:
// - Define data containers for CFD simulation results.
// - Header-only data structures for now (implementation minimal).
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
};

} // namespace lift::cfd
