#pragma once

#include <iosfwd>
#include <string>

#include "engine/analysis/closeout_types.hpp"

namespace lift {

struct JsonWriteOptions {
  // Pretty output = newlines + indentation
  bool pretty = true;
  int indent_spaces = 2;

  // JSON cannot represent NaN/Inf. If a numeric field is unset (NaN/Inf),
  // emit `null` instead of omitting the field.
  bool emit_null_for_unset = true;
};

// Serialize a CloseoutReport to JSON (stable key order, deterministic output).
std::string closeout_report_to_json(const CloseoutReport& report,
                                   const JsonWriteOptions& opt = {});

// Stream version (avoids extra copy).
void write_closeout_report_json(std::ostream& os,
                                const CloseoutReport& report,
                                const JsonWriteOptions& opt = {});

}  // namespace lift

