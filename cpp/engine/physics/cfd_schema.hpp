/*
===============================================================================
Fragment 3.2.10 — Schema Versioning + Deterministic Hash (Manifest/Results Audit) (C++)
File: cfd_schema.hpp
===============================================================================
*/

#pragma once

#include "bemt_require.hpp"

#include <cstdint>
#include <string>
#include <string_view>

namespace lift::bemt::cfd {

// 64-bit FNV-1a hash for deterministic content tagging.
std::uint64_t fnv1a64(std::string_view s) noexcept;

// Hex-encode a 64-bit value (lowercase, width=16, zero-padded).
std::string hex64(std::uint64_t h);

// Build an audit tag: "<schema_version>:<fnv_hex(content)>".
// schema_version must be non-empty.
std::string audit_tag(const char* schema_version, std::string_view content);

} // namespace lift::bemt::cfd
