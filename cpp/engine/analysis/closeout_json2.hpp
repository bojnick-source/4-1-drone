#pragma once

#include <string>

#include "engine/analysis/closeout_types.hpp"
#include "engine/analysis/closeout_error.hpp"

namespace lift {

// Serialize a CloseoutReport to JSON.
// - Deterministic field ordering
// - NaN/Inf are emitted as null (or omitted where appropriate)
// - Safe escaping
std::string closeout_to_json(const CloseoutReport& report);

// Optional convenience: write JSON to disk (throws lift::Error on failure).
void write_closeout_json_file(const CloseoutReport& report, const std::string& path);

}  // namespace lift
