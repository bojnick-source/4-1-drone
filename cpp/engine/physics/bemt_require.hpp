#pragma once
/*
===============================================================================
Fragment 3.1.09 — Hardened Math Utilities + Require Glue (C++)
File: bemt_require.hpp
===============================================================================
*/

#include "engine/physics/bemt_error.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace lift::bemt {

// -----------------------------
// Constants
// -----------------------------
inline constexpr double kPi = 3.141592653589793238462643383279502884;

// -----------------------------
// Finite checks
// -----------------------------
inline bool is_finite(double x) noexcept {
    return std::isfinite(x) != 0;
}

// -----------------------------
// Clamp (generic for arithmetic)
// -----------------------------
template <typename T>
inline constexpr T clamp(T v, T lo, T hi) noexcept {
    static_assert(std::is_arithmetic<T>::value, "clamp requires arithmetic type");
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// -----------------------------
// Safe division (never NaN/Inf)
// -----------------------------
inline double safe_div(double num, double den, double fallback = 0.0) noexcept {
    if (!is_finite(num) || !is_finite(den)) return fallback;
    if (den == 0.0) return fallback;
    const double q = num / den;
    return is_finite(q) ? q : fallback;
}

// -----------------------------
// Unit helpers
// -----------------------------
inline constexpr double deg2rad(double deg) noexcept { return deg * (kPi / 180.0); }
inline constexpr double rad2deg(double rad) noexcept { return rad * (180.0 / kPi); }

// -----------------------------
// Common numeric guards
// -----------------------------
inline double positive_or(double x, double fallback) noexcept {
    return (is_finite(x) && x > 0.0) ? x : fallback;
}

inline double nonneg_or(double x, double fallback) noexcept {
    return (is_finite(x) && x >= 0.0) ? x : fallback;
}

// -----------------------------
// Require wrappers (optional)
// Keep using LIFT_BEMT_REQUIRE macro for file/line context.
// -----------------------------
inline void require_finite(double x, ErrorCode code, const char* msg) {
    LIFT_BEMT_REQUIRE(is_finite(x), code, msg ? msg : "Non-finite value");
}

inline void require_positive(double x, ErrorCode code, const char* msg) {
    LIFT_BEMT_REQUIRE(is_finite(x) && x > 0.0, code, msg ? msg : "Non-positive value");
}

inline void require_nonnegative(double x, ErrorCode code, const char* msg) {
    LIFT_BEMT_REQUIRE(is_finite(x) && x >= 0.0, code, msg ? msg : "Negative value");
}

} // namespace lift::bemt
