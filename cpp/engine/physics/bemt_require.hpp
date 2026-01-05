#pragma once
// ============================================================================
// BEMT Requirements and Error Handling
// File: cpp/engine/physics/bemt_require.hpp
// ============================================================================
//
// Purpose:
// - Provide error codes and exception types for BEMT uncertainty analysis.
// - Consistent with existing lift::LiftError hierarchy.
//
// ============================================================================

#include "engine/core/errors.hpp"

#include <string>
#include <stdexcept>

namespace lift::bemt {

enum class ErrorCode {
    InvalidInput,
    NumericalFailure,
    IoError,
    ValidationFailure
};

class BemtException : public lift::LiftError {
public:
    BemtException(ErrorCode code, std::string msg)
        : LiftError(std::move(msg)), code_(code) {}

    ErrorCode code() const { return code_; }

private:
    ErrorCode code_;
};

} // namespace lift::bemt
