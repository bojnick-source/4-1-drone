/*
===============================================================================
Fragment 3.2.10 — Schema Versioning + Deterministic Hash (Manifest/Results Audit) (C++)
File: cfd_schema.cpp
===============================================================================
*/

#include "cfd_schema.hpp"

#include <sstream>
#include <iomanip>

namespace lift::bemt::cfd {

std::uint64_t fnv1a64(std::string_view s) noexcept {
    constexpr std::uint64_t FNV_OFFSET = 14695981039346656037ull;
    constexpr std::uint64_t FNV_PRIME  = 1099511628211ull;

    std::uint64_t h = FNV_OFFSET;
    for (unsigned char c : s) {
        h ^= static_cast<std::uint64_t>(c);
        h *= FNV_PRIME;
    }
    return h;
}

std::string hex64(std::uint64_t h) {
    std::ostringstream os;
    os.setf(std::ios::hex, std::ios::basefield);
    os << std::nouppercase << std::setfill('0') << std::setw(16) << h;
    return os.str();
}

std::string audit_tag(const char* schema_version, std::string_view content) {
    LIFT_BEMT_REQUIRE(schema_version && *schema_version, ::lift::bemt::ErrorCode::InvalidInput, "schema_version empty");
    const std::uint64_t h = fnv1a64(content);
    std::string out;
    out.reserve(64);
    out.append(schema_version);
    out.push_back(':');
    out.append(hex64(h));
    return out;
}

} // namespace lift::bemt::cfd
