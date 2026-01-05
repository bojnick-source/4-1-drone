/*
===============================================================================
Fragment 3.1.18 — Error System (ErrorCode + Exception + Require Macro)
File: bemt_error.hpp
===============================================================================
*/

#pragma once

#include <cstdint>
#include <exception>
#include <string>

namespace lift::bemt {

// Stable error codes for CSV + downstream tooling.
// Keep these values stable once public.
enum class ErrorCode : std::uint16_t {
  Ok = 0,

  // Input / config / data
  InvalidInput = 10,
  InvalidGeometry = 11,
  InvalidEnvironment = 12,
  InvalidOperatingPoint = 13,
  InvalidConfig = 14,

  MissingPolarData = 20,
  PolarOutOfRange = 21,
  OutOfRange = 22,

  // Solver / numerical
  DomainError = 30,
  NonConverged = 31,
  NumericalFailure = 32,

  // IO / export (future)
  IOError = 40,
  ParseError = 41
};

struct ErrorSite final {
  const char* file = "";
  const char* func = "";
  int line = 0;
};

// Exception type used by LIFT_BEMT_REQUIRE / fail().
class BemtError final : public std::exception {
public:
  BemtError(ErrorCode c, std::string msg, ErrorSite site = {})
      : code_(c), msg_(msg.empty() ? std::string{"<empty error message>"} : std::move(msg)), site_(site) {}

  BemtError(ErrorCode c, const char* msg, ErrorSite site = {})
      : code_(c),
        msg_(msg && msg[0] ? std::string{msg} : std::string{"<empty error message>"}),
        site_(site) {}

  const char* what() const noexcept override { return msg_.c_str(); }
  ErrorCode code() const noexcept { return code_; }
  const ErrorSite& where() const noexcept { return site_; }

private:
  ErrorCode code_;
  std::string msg_;
  ErrorSite site_;
};

// Helper to throw with site info.
[[noreturn]] inline void fail(ErrorCode code, const char* msg, ErrorSite site) {
  throw BemtError(code, msg ? msg : "BemtError", site);
}

} // namespace lift::bemt

// Site macro
#define LIFT_BEMT_SITE ::lift::bemt::ErrorSite{__FILE__, __func__, __LINE__}

// Require macro (non-negotiable hard fail for invalid states).
#define LIFT_BEMT_REQUIRE(cond, code, msg)     \
  do {                                         \
    if (!(cond)) {                             \
      ::lift::bemt::fail((code), (msg), LIFT_BEMT_SITE); \
    }                                          \
  } while (0)
