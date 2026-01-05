#pragma once

#include "engine/physics/bemt_solver.hpp"
#include "engine/physics/bemt_forward.hpp"

#include <cstddef>
#include <cstdint>
#include <list>
#include <mutex>
#include <optional>
#include <string>
#include <limits>
#include <unordered_map>
#include <utility>

namespace lift::bemt {

// Cache goal: accelerate optimization loops by memoizing deterministic BEMT evaluations.
// Hardened properties:
// - Deterministic key hashing (stable across processes for same numeric inputs)
// - Quantization to avoid cache misses from tiny floating jitter
// - Bounded memory via LRU eviction
// - Thread-safe option (mutex) for shared caches
// - Explicit "polar_id" so a changed airfoil table invalidates old entries

struct CacheQuantization final {
    // Quantization steps (choose conservative defaults; tune for your optimizer)
    double len_m = 1e-6;        // lengths (m)
    double ang_rad = 1e-7;      // angles (rad)
    double vel_mps = 1e-5;      // velocities (m/s)
    double omega_rad_s = 1e-5;  // angular speed (rad/s)
    double rho = 1e-6;          // density
    double mu = 1e-9;           // viscosity
    double tol = 1e-12;         // generic fallback

    void validate() const;
};

struct EvalCacheKey final {
    std::uint64_t h = 0;          // primary hash
    std::uint64_t h2 = 0;         // secondary hash for collision hardening
    std::uint32_t kind = 0;       // 0=hover/axial, 1=forward
    std::uint32_t reserved = 0;

    bool operator==(const EvalCacheKey& o) const noexcept {
        return h == o.h && h2 == o.h2 && kind == o.kind;
    }
};

struct EvalCacheKeyHash final {
    std::size_t operator()(const EvalCacheKey& k) const noexcept {
        // combine hashes
        return static_cast<std::size_t>(k.h ^ (k.h2 + 0x9e3779b97f4a7c15ULL + (k.h << 6) + (k.h >> 2)));
    }
};

struct CacheStats final {
    std::uint64_t hits = 0;
    std::uint64_t misses = 0;
    std::uint64_t inserts = 0;
    std::uint64_t evictions = 0;
};

class EvalCache final {
public:
    explicit EvalCache(std::size_t max_entries = 2048);

    void set_max_entries(std::size_t n);
    std::size_t max_entries() const noexcept;

    void clear();

    CacheStats stats() const;

    // Hover/axial evaluation cache
    std::optional<BemtResult> get_hover(const EvalCacheKey& key);
    void put_hover(const EvalCacheKey& key, BemtResult value);

    // Forward evaluation cache
    std::optional<ForwardResult> get_forward(const EvalCacheKey& key);
    void put_forward(const EvalCacheKey& key, ForwardResult value);

private:
    template <class T>
    struct Node final {
        EvalCacheKey key{};
        T value{};
    };

    template <class T>
    using ListT = std::list<Node<T>>;

    template <class T>
    using MapT = std::unordered_map<EvalCacheKey, typename ListT<T>::iterator, EvalCacheKeyHash>;

    template <class T>
    std::optional<T> get_(ListT<T>& lst, MapT<T>& mp, const EvalCacheKey& key);

    template <class T>
    void put_(ListT<T>& lst, MapT<T>& mp, const EvalCacheKey& key, T value);

private:
    mutable std::mutex mtx_;
    std::size_t max_entries_ = 2048;

    CacheStats stats_{};

    ListT<BemtResult> hover_list_;
    MapT<BemtResult>  hover_map_;

    ListT<ForwardResult> forward_list_;
    MapT<ForwardResult>  forward_map_;
};

// --------------------------
// Deterministic key building
// --------------------------
struct KeyBuilder final {
    CacheQuantization q{};

    // The polar_id should be stable across runs (e.g., "NACA0012@ReGridV3" or hash of file contents).
    std::string polar_id;

    explicit KeyBuilder(CacheQuantization q_ = {}, std::string polar_id_ = "")
        : q(std::move(q_)), polar_id(std::move(polar_id_)) {}

    EvalCacheKey make_hover_key(const RotorGeometry& g, const Environment& e,
                               const OperatingPoint& op, const SolverConfig& cfg) const;

    EvalCacheKey make_forward_key(const RotorGeometry& g, const Environment& e,
                                 const OperatingPoint& op, const SolverConfig& cfg,
                                 double V_inplane_mps, const ForwardConfig& fcfg) const;

    // Exposed for helper hash functions used by key builders.
    std::int64_t qd_(double v, double step, double fallback) const noexcept;

    void hash_string_(std::uint64_t& h1, std::uint64_t& h2, const std::string& s) const noexcept;
    void hash_u64_(std::uint64_t& h1, std::uint64_t& h2, std::uint64_t x) const noexcept;
    void hash_i64_(std::uint64_t& h1, std::uint64_t& h2, std::int64_t x) const noexcept;

private:
    static std::uint64_t fnv1a64_init() noexcept { return 1469598103934665603ULL; }
    static std::uint64_t fnv1a64_step(std::uint64_t h, std::uint64_t x) noexcept {
        h ^= x;
        h *= 1099511628211ULL;
        return h;
    }

    static std::uint64_t rotl64(std::uint64_t x, int r) noexcept {
        return (x << r) | (x >> (64 - r));
    }
};

} // namespace lift::bemt
