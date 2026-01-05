// ============================================================================
// Fragment 3.3.03 — Uncertainty Sampler (Monte Carlo Wrapper for Scalar Params)
// File: uncertainty.hpp
// ============================================================================
//
// Purpose:
// - Generate Monte Carlo samples for uncertain scalar inputs (fast, low-order).
// - Supports:
//    - Uniform(a,b)
//    - Normal(mu,sigma) with hard clamp
//    - Lognormal via (mu_ln, sigma_ln) with clamp
// - Deterministic RNG with explicit seed.
// - Returns sampled values and basic metadata for auditing.
//
// This sampler is used to perturb:
// - rho, mu (env)
// - omega, collective (op)
// - radius/chord scale factors (geom)
// - any scalar knobs you choose
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lift::bemt::prob {

enum class DistType : std::uint8_t {
    Uniform = 0,
    Normal = 1,
    LogNormal = 2
};

struct DistSpec final {
    DistType type = DistType::Uniform;

    // Parameters:
    // Uniform: a=p1, b=p2
    // Normal:  mu=p1, sigma=p2
    // LogNormal: mu_ln=p1, sigma_ln=p2  (exp(N(mu_ln,sigma_ln)))
    double p1 = 0.0;
    double p2 = 1.0;

    // Hard clamp (applied after sampling)
    double lo = -1e300;
    double hi =  1e300;

    void validate() const {
        LIFT_BEMT_REQUIRE(is_finite(p1) && is_finite(p2), ErrorCode::InvalidConfig, "DistSpec p1/p2 not finite");
        LIFT_BEMT_REQUIRE(is_finite(lo) && is_finite(hi) && lo < hi, ErrorCode::InvalidConfig, "DistSpec clamp invalid");

        if (type == DistType::Uniform) {
            LIFT_BEMT_REQUIRE(p1 < p2, ErrorCode::InvalidConfig, "Uniform requires a<b");
        } else if (type == DistType::Normal || type == DistType::LogNormal) {
            LIFT_BEMT_REQUIRE(p2 > 0.0, ErrorCode::InvalidConfig, "Normal/LogNormal require sigma>0");
        } else {
            LIFT_BEMT_REQUIRE(false, ErrorCode::InvalidConfig, "Unknown DistType");
        }
    }
};

struct SampleSet final {
    std::string name;
    std::uint64_t seed = 0;
    std::vector<double> x;

    void validate() const {
        LIFT_BEMT_REQUIRE(!name.empty(), ErrorCode::InvalidInput, "SampleSet.name empty");
        for (double v : x) LIFT_BEMT_REQUIRE(is_finite(v), ErrorCode::InvalidInput, "SampleSet has non-finite");
    }
};

// Deterministic RNG (xorshift64*).
class Rng64 final {
public:
    explicit Rng64(std::uint64_t seed) : s_(seed ? seed : 0x9e3779b97f4a7c15ull) {}

    std::uint64_t next_u64() noexcept {
        std::uint64_t x = s_;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        s_ = x;
        return x * 2685821657736338717ull;
    }

    // Uniform in [0,1)
    double next_u01() noexcept {
        // Take top 53 bits -> double mantissa
        const std::uint64_t u = next_u64();
        const std::uint64_t m = (u >> 11) | 1ull; // ensure nonzero
        return static_cast<double>(m) * (1.0 / 9007199254740992.0); // 2^53
    }

private:
    std::uint64_t s_;
};

struct SamplerConfig final {
    std::size_t n = 1000;
    std::uint64_t seed = 1;

    void validate() const {
        LIFT_BEMT_REQUIRE(n >= 1 && n <= 10'000'000, ErrorCode::InvalidConfig, "SamplerConfig n invalid");
    }
};

// Sample a distribution spec into a SampleSet.
SampleSet sample(const std::string& name, const DistSpec& spec, const SamplerConfig& cfg);

// Helpers for common specs
DistSpec uniform(double a, double b, double lo, double hi);
DistSpec normal(double mu, double sigma, double lo, double hi);
DistSpec lognormal(double mu_ln, double sigma_ln, double lo, double hi);

} // namespace lift::bemt::prob
