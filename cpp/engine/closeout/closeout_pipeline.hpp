#pragma once
// ============================================================================
// Closeout Pipeline
// File: cpp/engine/closeout/closeout_pipeline.hpp
// ============================================================================
//
// Purpose:
// - Define input/output structures for closeout pipeline.
// - Minimal working skeleton for compilation.
//
// ============================================================================

#include <string>
#include <vector>

namespace lift::closeout {

struct EvidenceItem {
    std::string key;
    std::string value;
    std::string units;
    std::string source;
    std::string notes;
};

struct GateCheck {
    std::string id;
    bool pass = false;
    double value = 0.0;
    double threshold = 0.0;
    std::string note;
};

struct GateResults {
    std::vector<GateCheck> checks;
};

struct CloseoutInput {
    // Placeholder for future input parameters
    void validate() const {}
};

struct CloseoutOutput {
    std::vector<EvidenceItem> evidence;
    GateResults gate;

    void validate() const {
        // Placeholder validation
    }
};

} // namespace lift::closeout
