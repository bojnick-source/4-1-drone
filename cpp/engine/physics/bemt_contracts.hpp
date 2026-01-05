// ============================================================================
// Fragment 3.1.14 — BEMT Contracts & Runtime Validation (Geometry/Op/AirState Guards + Member-Detection) (C++)
// File: bemt_contracts.hpp
// ============================================================================
//
// Purpose:
// - Harden the BEMT pipeline by enforcing consistent runtime validation of inputs.
// - Uses C++20 member-detection (requires) so it can adapt to your existing structs.
// - Designed to be called at the start of bemt_solver / bemt_forward / bemt_mc entrypoints.
//
// What it validates (when fields exist):
// - RotorGeometry.stations: size >= 2, strictly increasing r_m, sane chord/twist
// - RotorGeometry: Rhub/Rtip if present (or derives from stations)
// - RotorOp: omega/collective if present
// - AirState: rho/mu if present
//
// Notes:
// - If a required structural member is missing, compilation will fail with a clear message.
// - Otherwise, checks are cheap and deterministic.
//
// ============================================================================

#pragma once
#include "bemt_require.hpp"
#include "bemt_safety.hpp"

#include <type_traits>
#include <vector>
#include <string>
#include <limits>
#include <cstddef>

namespace lift::bemt::contracts {

// -----------------------------
// Member detection helpers (C++20)
// -----------------------------

template <class T>
concept HasStations = requires(const T& t) { t.stations; };

template <class T>
concept HasRM = requires(const T& t) { t.r_m; };

template <class T>
concept HasChord = requires(const T& t) { t.chord_m; };

template <class T>
concept HasTwist = requires(const T& t) { t.twist_rad; };

template <class T>
concept HasRhub = requires(const T& t) { t.r_hub_m; } || requires(const T& t) { t.rhub_m; } || requires(const T& t) { t.Rhub_m; };

template <class T>
concept HasRtip = requires(const T& t) { t.r_tip_m; } || requires(const T& t) { t.rtip_m; } || requires(const T& t) { t.Rtip_m; };

template <class T>
concept HasBlades = requires(const T& t) { t.blades; } || requires(const T& t) { t.B; };

template <class T>
concept HasOmega = requires(const T& t) { t.omega_rad_s; } || requires(const T& t) { t.omega; };

template <class T>
concept HasCollective = requires(const T& t) { t.collective_rad; } || requires(const T& t) { t.collective; };

template <class T>
concept HasRho = requires(const T& t) { t.rho_kg_m3; } || requires(const T& t) { t.rho; };

template <class T>
concept HasMu = requires(const T& t) { t.mu_Pa_s; } || requires(const T& t) { t.mu; };

// -----------------------------
// Field accessors (adapt to your naming)
// -----------------------------

template <class Station>
requires HasRM<Station>
inline double station_r(const Station& s) noexcept { return static_cast<double>(s.r_m); }

template <class Station>
inline double station_r(const Station&) noexcept {
    static_assert(HasRM<Station>, "Station must expose r_m (meters).");
    return 0.0;
}

template <class Station>
inline double station_chord(const Station& s) noexcept {
    if constexpr (HasChord<Station>) return static_cast<double>(s.chord_m);
    return std::numeric_limits<double>::quiet_NaN();
}

template <class Station>
inline double station_twist(const Station& s) noexcept {
    if constexpr (HasTwist<Station>) return static_cast<double>(s.twist_rad);
    return std::numeric_limits<double>::quiet_NaN();
}

template <class G>
inline double geom_rhub(const G& g) noexcept {
    if constexpr (requires(const G& x){ x.r_hub_m; }) return static_cast<double>(g.r_hub_m);
    else if constexpr (requires(const G& x){ x.rhub_m; }) return static_cast<double>(g.rhub_m);
    else if constexpr (requires(const G& x){ x.Rhub_m; }) return static_cast<double>(g.Rhub_m);
    else return std::numeric_limits<double>::quiet_NaN();
}

template <class G>
inline double geom_rtip(const G& g) noexcept {
    if constexpr (requires(const G& x){ x.r_tip_m; }) return static_cast<double>(g.r_tip_m);
    else if constexpr (requires(const G& x){ x.rtip_m; }) return static_cast<double>(g.rtip_m);
    else if constexpr (requires(const G& x){ x.Rtip_m; }) return static_cast<double>(g.Rtip_m);
    else return std::numeric_limits<double>::quiet_NaN();
}

template <class Op>
inline double op_omega(const Op& op) noexcept {
    if constexpr (requires(const Op& x){ x.omega_rad_s; }) return static_cast<double>(op.omega_rad_s);
    else if constexpr (requires(const Op& x){ x.omega; }) return static_cast<double>(op.omega);
    else return std::numeric_limits<double>::quiet_NaN();
}

template <class Op>
inline double op_collective(const Op& op) noexcept {
    if constexpr (requires(const Op& x){ x.collective_rad; }) return static_cast<double>(op.collective_rad);
    else if constexpr (requires(const Op& x){ x.collective; }) return static_cast<double>(op.collective);
    else return std::numeric_limits<double>::quiet_NaN();
}

template <class Air>
inline double air_rho(const Air& a) noexcept {
    if constexpr (requires(const Air& x){ x.rho_kg_m3; }) return static_cast<double>(a.rho_kg_m3);
    else if constexpr (requires(const Air& x){ x.rho; }) return static_cast<double>(a.rho);
    else return std::numeric_limits<double>::quiet_NaN();
}

template <class Air>
inline double air_mu(const Air& a) noexcept {
    if constexpr (requires(const Air& x){ x.mu_Pa_s; }) return static_cast<double>(a.mu_Pa_s);
    else if constexpr (requires(const Air& x){ x.mu; }) return static_cast<double>(a.mu);
    else return std::numeric_limits<double>::quiet_NaN();
}

// -----------------------------
// Validation (runtime)
// -----------------------------

struct ContractConfig final {
    bool require_strict_r = true;
    bool require_positive_chord = true;

    // Sane bounds (guard rails)
    double chord_min_m = 1e-4;
    double chord_max_m = 5.0;

    double rho_min = 0.05;
    double rho_max = 5.0;

    double mu_min = 1e-7;
    double mu_max = 1e-2;

    int blades_min = 2;
    int blades_max = 12;

    void validate() const {
        LIFT_BEMT_REQUIRE(chord_min_m > 0.0 && chord_max_m > chord_min_m, ErrorCode::InvalidConfig, "ContractConfig chord bounds invalid");
        LIFT_BEMT_REQUIRE(rho_min > 0.0 && rho_max > rho_min, ErrorCode::InvalidConfig, "ContractConfig rho bounds invalid");
        LIFT_BEMT_REQUIRE(mu_min > 0.0 && mu_max > mu_min, ErrorCode::InvalidConfig, "ContractConfig mu bounds invalid");
        LIFT_BEMT_REQUIRE(blades_min >= 2 && blades_max >= blades_min, ErrorCode::InvalidConfig, "ContractConfig blades bounds invalid");
    }
};

template <class Geometry>
inline void validate_geometry(const Geometry& g, const ContractConfig& cfg_in = {}) {
    ContractConfig cfg = cfg_in;
    cfg.validate();

    static_assert(HasStations<Geometry>, "RotorGeometry must expose .stations (vector of stations).");

    const auto& st = g.stations;
    LIFT_BEMT_REQUIRE(st.size() >= 2, ErrorCode::InvalidInput, "geometry: stations.size() < 2");

    // Derive Rhub/Rtip if not explicit
    double Rhub = geom_rhub(g);
    double Rtip = geom_rtip(g);

    // Validate station radii monotonic
    double prev_r = station_r(st[0]);
    LIFT_BEMT_REQUIRE(is_finite(prev_r) && prev_r >= 0.0, ErrorCode::InvalidInput, "geometry: station r invalid");

    for (std::size_t i = 1; i < st.size(); ++i) {
        const double r = station_r(st[i]);
        LIFT_BEMT_REQUIRE(is_finite(r) && r >= 0.0, ErrorCode::InvalidInput, "geometry: station r invalid");
        if (cfg.require_strict_r) {
            LIFT_BEMT_REQUIRE(r > prev_r, ErrorCode::InvalidInput, "geometry: station r must be strictly increasing");
        } else {
            LIFT_BEMT_REQUIRE(r >= prev_r, ErrorCode::InvalidInput, "geometry: station r must be non-decreasing");
        }
        prev_r = r;

        // Optional chord checks
        if (cfg.require_positive_chord) {
            const double c = station_chord(st[i]);
            if (is_finite(c)) {
                LIFT_BEMT_REQUIRE(c >= cfg.chord_min_m && c <= cfg.chord_max_m,
                                  ErrorCode::InvalidInput, "geometry: chord out of bounds");
            }
        }
    }

    const double r0 = station_r(st.front());
    const double rN = station_r(st.back());

    if (is_finite(Rhub)) {
        LIFT_BEMT_REQUIRE(Rhub >= 0.0 && Rhub <= r0 + 1e-9, ErrorCode::InvalidInput, "geometry: Rhub must be <= first station r");
    } else {
        Rhub = r0;
    }

    if (is_finite(Rtip)) {
        LIFT_BEMT_REQUIRE(Rtip >= rN - 1e-9, ErrorCode::InvalidInput, "geometry: Rtip must be >= last station r");
    } else {
        Rtip = rN;
    }

    LIFT_BEMT_REQUIRE(Rtip > Rhub, ErrorCode::InvalidInput, "geometry: Rtip must be > Rhub");
}

template <class Op>
inline void validate_op(const Op& op, const ContractConfig& cfg_in = {}) {
    (void)cfg_in;
    // omega if present
    const double w = op_omega(op);
    if (is_finite(w)) {
        LIFT_BEMT_REQUIRE(w >= 0.0 && w <= 1e6, ErrorCode::InvalidInput, "op: omega out of bounds");
    }
    const double coll = op_collective(op);
    if (is_finite(coll)) {
        LIFT_BEMT_REQUIRE(std::fabs(coll) <= 3.14159265358979323846, ErrorCode::InvalidInput, "op: collective magnitude too large");
    }
}

template <class Air>
inline void validate_air(const Air& air, const ContractConfig& cfg_in = {}) {
    ContractConfig cfg = cfg_in;
    cfg.validate();

    const double rho = air_rho(air);
    if (is_finite(rho)) {
        LIFT_BEMT_REQUIRE(rho >= cfg.rho_min && rho <= cfg.rho_max, ErrorCode::InvalidInput, "air: rho out of bounds");
    }

    const double mu = air_mu(air);
    if (is_finite(mu)) {
        LIFT_BEMT_REQUIRE(mu >= cfg.mu_min && mu <= cfg.mu_max, ErrorCode::InvalidInput, "air: mu out of bounds");
    }
}

template <class Geometry>
inline double rotor_radius_tip(const Geometry& g) {
    static_assert(HasStations<Geometry>, "RotorGeometry must expose .stations");
    const double R = geom_rtip(g);
    if (is_finite(R) && R > 0.0) return R;
    return station_r(g.stations.back());
}

} // namespace lift::bemt::contracts
