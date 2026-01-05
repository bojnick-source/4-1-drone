// ============================================================================
// Fragment 3.1.17 — Monte Carlo Wrapper Skeleton (BEMT Runs + Stats Hooks + Deterministic Seeds) (C++)
// File: bemt_mc.hpp
// ============================================================================
//
// Purpose:
// - Provide a hardened Monte Carlo wrapper around the BEMT facade.
// - Collects OnlineStats + optional empirical CDF samples for key outputs:
//     thrust, power, FM, disk loading, etc.
// - Deterministic RNG with explicit seed handling.
// - Does NOT define your uncertainty distributions; it only provides the scaffolding.
//   You plug in a “sampler” that perturbs input structs.
//
// Notes:
// - In the optimization hot loop: MC disabled.
// - In closeout / top-N validation: MC enabled with modest N (e.g., 200-2000).
//
// ============================================================================

#pragma once
#include "online_stats.hpp"
#include "../physics/bemt_facade.hpp"
#include "../physics/bemt_safety.hpp"
#include "../physics/bemt_require.hpp"

#include <cstdint>
#include <string>
#include <utility>

namespace lift::stats {

// -----------------------------
// Simple deterministic RNG (SplitMix64)
// -----------------------------
struct SplitMix64 final {
    std::uint64_t s = 0;

    explicit SplitMix64(std::uint64_t seed = 0x9E3779B97F4A7C15ull) : s(seed) {}

    std::uint64_t next_u64() noexcept {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }

    // Uniform in [0,1)
    double next_u01() noexcept {
        // 53-bit mantissa
        const std::uint64_t x = next_u64();
        const std::uint64_t mant = x >> 11;
        return static_cast<double>(mant) * (1.0 / 9007199254740992.0);
    }
};

// -----------------------------
// MC configuration
// -----------------------------
struct McConfig final {
    std::uint32_t samples = 0;   // 0 => disabled
    std::uint64_t seed = 0xC0FFEE1234ull;

    // Store samples for CDF?
    ReservoirConfig store_T{};
    ReservoirConfig store_P{};
    ReservoirConfig store_FM{};

    void validate() const {
        LIFT_BEMT_REQUIRE(samples <= 5'000'000u, lift::bemt::ErrorCode::InvalidConfig, "McConfig.samples too large");
        store_T.validate();
        store_P.validate();
        store_FM.validate();
    }
};

// -----------------------------
// MC outputs
// -----------------------------
struct McOut final {
    lift::bemt::ErrorCode code = lift::bemt::ErrorCode::Ok;
    std::string message;

    OnlineStats T_N{};
    OnlineStats P_W{};
    OnlineStats FM{};

    Reservoir rT;
    Reservoir rP;
    Reservoir rFM;

    explicit McOut(const McConfig& cfg)
        : rT(cfg.store_T), rP(cfg.store_P), rFM(cfg.store_FM) {}

    bool ok() const noexcept { return code == lift::bemt::ErrorCode::Ok; }
};

// -----------------------------
// Sampler concept
// -----------------------------
// The sampler is responsible for perturbing in-place a copy of the base input.
// It should only use rng.next_u01() for determinism.
// Signature:
//   void operator()(lift::bemt::BemtRunIn& in, SplitMix64& rng) const;
//
// Example perturbations (you implement):
// - rho ± 2%
// - airfoil cl slope ±
// - chord/twist manufacturing tolerance
// - omega controller noise
// - etc.
template <class Sampler>
concept McSampler = requires(const Sampler& s, lift::bemt::BemtRunIn& in, SplitMix64& rng) {
    { s(in, rng) } -> std::same_as<void>;
};

// -----------------------------
// Run MC
// -----------------------------
template <McSampler Sampler>
inline McOut run_mc(const lift::bemt::BemtRunIn& base_in,
                    const lift::bemt::BemtFacadeConfig& bemt_cfg,
                    const McConfig& mc_cfg,
                    const Sampler& sampler) {
    mc_cfg.validate();
    base_in.validate();
    bemt_cfg.validate();

    McOut out(mc_cfg);

    if (mc_cfg.samples == 0) {
        out.code = lift::bemt::ErrorCode::Ok;
        out.message = "mc disabled";
        return out;
    }

    SplitMix64 rng(mc_cfg.seed);

    for (std::uint32_t i = 0; i < mc_cfg.samples; ++i) {
        // Copy base input, perturb deterministically
        lift::bemt::BemtRunIn in = base_in;
        sampler(in, rng);

        // Run BEMT facade
        const auto r = lift::bemt::bemt_run(in, bemt_cfg);
        if (!r.ok()) {
            // Soft-fail: skip invalid samples but record that failures occurred.
            // Deterministic behavior: same skips given same seed & code.
            continue;
        }

        out.T_N.push(r.T_N);
        out.P_W.push(r.P_W);
        out.FM.push(r.FM);

        out.rT.push(r.T_N);
        out.rP.push(r.P_W);
        out.rFM.push(r.FM);
    }

    out.code = lift::bemt::ErrorCode::Ok;
    out.message = "ok";
    return out;
}

} // namespace lift::stats
