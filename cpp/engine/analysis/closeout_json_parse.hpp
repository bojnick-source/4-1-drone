#pragma once

#include <cstddef>
#include <iosfwd>
#include <string>
#include <string_view>

#include "engine/analysis/closeout_types.hpp"

namespace lift {

struct JsonParseError {
  std::string message;
  size_t offset = 0;  // byte offset in input
  int line = 1;       // 1-based
  int col = 1;        // 1-based
};

/// Parse CloseoutReport from JSON text.
/// - Accepts `null` for unset numeric fields (converts to NaN internally).
/// - Rejects NaN/Inf numeric literals (not valid JSON).
/// - Enforces expected schema types; unknown keys are ignored (forward compatible).
bool parse_closeout_report_json(std::string_view json,
                               CloseoutReport* out,
                               JsonParseError* err = nullptr);

/// Stream convenience (reads full stream into memory).
bool parse_closeout_report_json(std::istream& is,
                               CloseoutReport* out,
                               JsonParseError* err = nullptr);

}  // namespace lift

