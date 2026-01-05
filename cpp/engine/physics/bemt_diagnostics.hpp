// ============================================================================
// Fragment 3.1.10 — BEMT Diagnostics (Clamp Flags, Convergence Flags, Deterministic Reason Codes) (C++)
// File: bemt_diagnostics.hpp
// ============================================================================
//
// Purpose:
// - Provide a single, deterministic “diagnostics record” for each BEMT solve.
// - Standardize: clamp flags, numerical guard trips, convergence outcomes, integrity failures.
// - Designed for closeout.csv / prob_closeout.csv hooks (no string spam in hot loops).
//
// Output style:
// - Compact bitmask flags + short reason string only at the end.
// - Caller can embed this into existing BemtResult or closeout row.
//
// ============================================================================

#pragma once
#include "bemt_error.hpp"
#include "bemt_safety.hpp"
#include "bemt_num_limits.hpp"
#include "bemt_integrity.hpp"

#include <cstdint>
#include <string>

namespace lift::bemt {

enum class DiagFlag : std::uint32_t {
    None                = 0u,

    // Clamp events
    ClampedPhi          = 1u << 0,
    ClampedAlpha        = 1u << 1,
    ClampedRe           = 1u << 2,
    ClampedLossF        = 1u << 3,
    ClampedPower        = 1u << 4,
    ClampedTorque       = 1u << 5,
    ClampedThrust       = 1u << 6,
    ClampedOther        = 1u << 7,

    // Numerical events
    HitEpsDivGuard      = 1u << 8,
    HitSqrtGuard        = 1u << 9,
    HitExpGuard         = 1u << 10,
    NonFiniteRecovered  = 1u << 11,

    // Convergence events
    NotConverged        = 1u << 12,
    Diverged            = 1u << 13,
    BracketFailed       = 1u << 14,

    // Integrity events
    TorquePowerMismatch = 1u << 15,
    FMOutOfBounds       = 1u << 16,
    NegativeThrust      = 1u << 17,
    NegativePower       = 1u << 18,

    // Data issues
    AirfoilOORAlpha     = 1u << 19,
    AirfoilOORRe        = 1u << 20,
    BadStations         = 1u << 21,

    // Reserved
    Reserved22          = 1u << 22,
    Reserved23          = 1u << 23,
    Reserved24          = 1u << 24,
    Reserved25          = 1u << 25,
    Reserved26          = 1u << 26,
    Reserved27          = 1u << 27,
    Reserved28          = 1u << 28,
    Reserved29          = 1u << 29,
    Reserved30          = 1u << 30,
    Reserved31          = 1u << 31
};

inline constexpr DiagFlag operator|(DiagFlag a, DiagFlag b) noexcept {
    return static_cast<DiagFlag>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}
inline constexpr DiagFlag operator&(DiagFlag a, DiagFlag b) noexcept {
    return static_cast<DiagFlag>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
}
inline DiagFlag& operator|=(DiagFlag& a, DiagFlag b) noexcept { a = a | b; return a; }

inline bool any(DiagFlag f) noexcept { return static_cast<std::uint32_t>(f) != 0u; }
inline bool has(DiagFlag mask, DiagFlag bit) noexcept { return any(mask & bit); }

struct BemtDiagnostics final {
    ErrorCode code = ErrorCode::Ok;

    // Bitmask flags
    DiagFlag flags = DiagFlag::None;

    // Small numeric fields that help debugging without huge logs
    int iters = 0;
    double last_rel_err = 0.0;
    double last_abs_err = 0.0;

    // Integrity snapshot (optional)
    double fm = 0.0;
    double disk_loading_N_m2 = 0.0;
    double torque_power_rel_err = 0.0;

    // Final short reason (optional; safe to leave empty in inner loops)
    std::string reason;

    bool ok() const noexcept { return code == ErrorCode::Ok; }

    void add_flag(DiagFlag f) noexcept { flags |= f; }
};

// Helper: add clamp flags based on named clamp events
inline void diag_set_clamp_flag(BemtDiagnostics& d, const char* what) noexcept {
    if (!what) { d.add_flag(DiagFlag::ClampedOther); return; }
    // Fast keyword matching; keep deterministic and cheap.
    const std::string s(what);
    if (s.find("phi") != std::string::npos)        d.add_flag(DiagFlag::ClampedPhi);
    else if (s.find("alpha") != std::string::npos) d.add_flag(DiagFlag::ClampedAlpha);
    else if (s.find("Re") != std::string::npos)    d.add_flag(DiagFlag::ClampedRe);
    else if (s.find("loss") != std::string::npos || s.find("F") != std::string::npos) d.add_flag(DiagFlag::ClampedLossF);
    else if (s.find("power") != std::string::npos) d.add_flag(DiagFlag::ClampedPower);
    else if (s.find("torque") != std::string::npos)d.add_flag(DiagFlag::ClampedTorque);
    else if (s.find("thrust") != std::string::npos)d.add_flag(DiagFlag::ClampedThrust);
    else                                           d.add_flag(DiagFlag::ClampedOther);
}

// Apply numeric limits to a few common outputs and record which clamps fired.
// This is meant as a post-pass just before closeout export.
struct ClampPostPassOut final {
    double T_N = 0.0;
    double Q_Nm = 0.0;
    double P_W = 0.0;
};

inline ClampPostPassOut clamp_postpass(double T_N, double Q_Nm, double P_W,
                                       const BemtNumLimits& lim,
                                       BemtDiagnostics& diag) {
    lim.validate();

    ClampPostPassOut o;
    bool c = false;

    o.T_N = clamp_thrust(T_N, lim, c);
    if (c) diag.add_flag(DiagFlag::ClampedThrust);

    o.Q_Nm = clamp_torque(Q_Nm, lim, c);
    if (c) diag.add_flag(DiagFlag::ClampedTorque);

    o.P_W = clamp_power(P_W, lim, c);
    if (c) diag.add_flag(DiagFlag::ClampedPower);

    return o;
}

// Run integrity checks and map failures to diagnostic flags.
// If disk_area_m2 <= 0, FM/DL checks are skipped.
inline void attach_integrity(BemtDiagnostics& d,
                             double T_N, double Q_Nm, double P_W,
                             double rho_kg_m3, double disk_area_m2, double omega_rad_s,
                             const IntegrityConfig& cfg = {}) {
    const auto in = bemt_integrity(T_N, Q_Nm, P_W, rho_kg_m3, disk_area_m2, omega_rad_s, cfg);

    d.disk_loading_N_m2 = in.disk_loading_N_m2;
    d.fm = in.FM;
    d.torque_power_rel_err = in.torque_power_rel_err;

    if (in.code != ErrorCode::Ok) {
        d.code = in.code;
        d.reason = in.message;

        if (in.message.find("power mismatch") != std::string::npos) d.add_flag(DiagFlag::TorquePowerMismatch);
        if (in.message.find("FM out of bounds") != std::string::npos) d.add_flag(DiagFlag::FMOutOfBounds);
        if (in.message.find("negative thrust") != std::string::npos) d.add_flag(DiagFlag::NegativeThrust);
        if (in.message.find("negative power") != std::string::npos) d.add_flag(DiagFlag::NegativePower);
    }
}

// Deterministic “summary string” for CSV closeout, compact.
inline std::string diag_summary(const BemtDiagnostics& d) {
    // Keep it stable: code|iters|flags|reason
    // reason truncated to avoid bloating CSV rows.
    std::string r = d.reason;
    if (r.size() > 96) r.resize(96);

    return std::to_string(static_cast<unsigned>(d.code)) + "|" +
           std::to_string(d.iters) + "|" +
           std::to_string(static_cast<std::uint32_t>(d.flags)) + "|" +
           r;
}

} // namespace lift::bemt
