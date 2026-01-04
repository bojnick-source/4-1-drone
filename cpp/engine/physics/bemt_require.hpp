#pragma once
// ============================================================================
// BEMT Require - Exception handling for BEMT subsystem
// File: cpp/engine/physics/bemt_require.hpp
// ============================================================================

#include "engine/core/errors.hpp"
#include "engine/analysis/closeset_types.hpp"

#include <string>
#include <stdexcept>

namespace lift::bemt {

// BEMT-specific exception that wraps lift::LiftError
class BemtException : public lift::LiftError {
public:
    BemtException(lift::ErrorCode code, std::string msg)
        : lift::LiftError(std::move(msg)), error_code_(code) {}
    
    lift::ErrorCode error_code() const { return error_code_; }
    
private:
    lift::ErrorCode error_code_;
};

// Alias ErrorCode from closeset_types for convenience
using ErrorCode = lift::ErrorCode;

} // namespace lift::bemt
