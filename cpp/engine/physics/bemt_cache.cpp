#include "engine/physics/bemt_cache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lift::bemt {

// ---------------- CacheQuantization ----------------
void CacheQuantization::validate() const {
    if (len_m <= 0.0 || ang_rad <= 0.0 || vel_mps <= 0.0 || omega_rad_s <= 0.0 ||
        rho <= 0.0 || mu <= 0.0 || tol <= 0.0) {
        throw BemtError(ErrorCode::InvalidInput, "CacheQuantization: non-positive step");
    }
}

// ---------------- EvalCache ----------------
EvalCache::EvalCache(std::size_t max_entries) : max_entries_(max_entries) {}

void EvalCache::set_max_entries(std::size_t n) {
    std::lock_guard<std::mutex> lk(mtx_);
    max_entries_ = std::max<std::size_t>(1, n);
}

std::size_t EvalCache::max_entries() const noexcept {
    std::lock_guard<std::mutex> lk(mtx_);
    return max_entries_;
}

void EvalCache::clear() {
    std::lock_guard<std::mutex> lk(mtx_);
    hover_list_.clear();
    hover_map_.clear();
    forward_list_.clear();
    forward_map_.clear();
    stats_ = CacheStats{};
}

CacheStats EvalCache::stats() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return stats_;
}

std::optional<BemtResult> EvalCache::get_hover(const EvalCacheKey& key) {
    std::lock_guard<std::mutex> lk(mtx_);
    return get_(hover_list_, hover_map_, key);
}

void EvalCache::put_hover(const EvalCacheKey& key, BemtResult value) {
    std::lock_guard<std::mutex> lk(mtx_);
    put_(hover_list_, hover_map_, key, std::move(value));
}

std::optional<ForwardResult> EvalCache::get_forward(const EvalCacheKey& key) {
    std::lock_guard<std::mutex> lk(mtx_);
    return get_(forward_list_, forward_map_, key);
}

void EvalCache::put_forward(const EvalCacheKey& key, ForwardResult value) {
    std::lock_guard<std::mutex> lk(mtx_);
    put_(forward_list_, forward_map_, key, std::move(value));
}

template <class T>
std::optional<T> EvalCache::get_(ListT<T>& lst, MapT<T>& mp, const EvalCacheKey& key) {
    auto it = mp.find(key);
    if (it == mp.end()) {
        ++stats_.misses;
        return std::nullopt;
    }
    // move to front (LRU)
    lst.splice(lst.begin(), lst, it->second);
    ++stats_.hits;
    return it->second->value;
}

template <class T>
void EvalCache::put_(ListT<T>& lst, MapT<T>& mp, const EvalCacheKey& key, T value) {
    auto it = mp.find(key);
    if (it != mp.end()) {
        // update existing, move to front
        it->second->value = std::move(value);
        lst.splice(lst.begin(), lst, it->second);
        return;
    }

    lst.push_front(Node<T>{key, std::move(value)});
    mp[key] = lst.begin();
    ++stats_.inserts;

    while (lst.size() > max_entries_) {
        auto last_it = std::prev(lst.end());
        mp.erase(last_it->key);
        lst.pop_back();
        ++stats_.evictions;
    }
}

// Explicit instantiations for the template helpers
template std::optional<BemtResult> EvalCache::get_<BemtResult>(ListT<BemtResult>&, MapT<BemtResult>&, const EvalCacheKey&);
template void EvalCache::put_<BemtResult>(ListT<BemtResult>&, MapT<BemtResult>&, const EvalCacheKey&, BemtResult);
template std::optional<ForwardResult> EvalCache::get_<ForwardResult>(ListT<ForwardResult>&, MapT<ForwardResult>&, const EvalCacheKey&);
template void EvalCache::put_<ForwardResult>(ListT<ForwardResult>&, MapT<ForwardResult>&, const EvalCacheKey&, ForwardResult);

// ---------------- KeyBuilder ----------------
std::int64_t KeyBuilder::qd_(double v, double step, double fallback) const noexcept {
    if (!std::isfinite(v) || step <= 0.0) return static_cast<std::int64_t>(std::llround(fallback / (step > 0.0 ? step : 1.0)));
    return static_cast<std::int64_t>(std::llround(v / step));
}

void KeyBuilder::hash_string_(std::uint64_t& h1, std::uint64_t& h2, const std::string& s) const noexcept {
    for (unsigned char c : s) {
        h1 = fnv1a64_step(h1, static_cast<std::uint64_t>(c));
        h2 = fnv1a64_step(h2, rotl64(static_cast<std::uint64_t>(c), 7));
    }
}

void KeyBuilder::hash_u64_(std::uint64_t& h1, std::uint64_t& h2, std::uint64_t x) const noexcept {
    h1 = fnv1a64_step(h1, x);
    h2 = fnv1a64_step(h2, rotl64(x, 13));
}

void KeyBuilder::hash_i64_(std::uint64_t& h1, std::uint64_t& h2, std::int64_t x) const noexcept {
    hash_u64_(h1, h2, static_cast<std::uint64_t>(x));
}

static void hash_stations(std::uint64_t& h1, std::uint64_t& h2, const RotorGeometry& g, const CacheQuantization& q, const KeyBuilder& kb) {
    kb.hash_i64_(h1, h2, kb.qd_(g.blade_count, 1.0, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(g.radius_m, q.len_m, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(g.hub_radius_m, q.len_m, 0.0));
    kb.hash_i64_(h1, h2, static_cast<std::int64_t>(g.tip_loss));
    kb.hash_i64_(h1, h2, static_cast<std::int64_t>(g.stations.size()));
    for (const auto& s : g.stations) {
        kb.hash_i64_(h1, h2, kb.qd_(s.r_m, q.len_m, 0.0));
        kb.hash_i64_(h1, h2, kb.qd_(s.chord_m, q.len_m, 0.0));
        kb.hash_i64_(h1, h2, kb.qd_(s.twist_rad, q.ang_rad, 0.0));
    }
}

static void hash_env(std::uint64_t& h1, std::uint64_t& h2, const Environment& e, const CacheQuantization& q, const KeyBuilder& kb) {
    kb.hash_i64_(h1, h2, kb.qd_(e.rho, q.rho, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(e.mu, q.mu, 0.0));
}

static void hash_op(std::uint64_t& h1, std::uint64_t& h2, const OperatingPoint& op, const CacheQuantization& q, const KeyBuilder& kb) {
    kb.hash_i64_(h1, h2, kb.qd_(op.V_inf, q.vel_mps, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(op.omega_rad_s, q.omega_rad_s, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(op.collective_offset_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, op.target_thrust_N.has_value() ? 1 : 0);
    if (op.target_thrust_N.has_value()) {
        kb.hash_i64_(h1, h2, kb.qd_(*op.target_thrust_N, q.len_m, 0.0));
    }
}

static void hash_cfg(std::uint64_t& h1, std::uint64_t& h2, const SolverConfig& cfg, const CacheQuantization& q, const KeyBuilder& kb) {
    kb.hash_i64_(h1, h2, static_cast<std::int64_t>(cfg.max_iter_inflow));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.tol_inflow, q.tol, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.inflow_relax, q.tol, 0.0));
    kb.hash_i64_(h1, h2, static_cast<std::int64_t>(cfg.max_iter_trim));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.tol_trim_N, q.tol, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.collective_min_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.collective_max_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.min_phi_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.max_phi_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.max_aoa_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.min_aoa_rad, q.ang_rad, 0.0));
    kb.hash_i64_(h1, h2, kb.qd_(cfg.min_dr_m, q.len_m, 0.0));
}

EvalCacheKey KeyBuilder::make_hover_key(const RotorGeometry& g, const Environment& e,
                                       const OperatingPoint& op, const SolverConfig& cfg) const {
    q.validate();
    std::uint64_t h1 = fnv1a64_init();
    std::uint64_t h2 = fnv1a64_init();

    hash_string_(h1, h2, polar_id);
    hash_stations(h1, h2, g, q, *this);
    hash_env(h1, h2, e, q, *this);
    hash_op(h1, h2, op, q, *this);
    hash_cfg(h1, h2, cfg, q, *this);

    EvalCacheKey k;
    k.kind = 0;
    k.h = h1;
    k.h2 = h2;
    return k;
}

EvalCacheKey KeyBuilder::make_forward_key(const RotorGeometry& g, const Environment& e,
                                         const OperatingPoint& op, const SolverConfig& cfg,
                                         double V_inplane_mps, const ForwardConfig& fcfg) const {
    q.validate();
    fcfg.validate();

    std::uint64_t h1 = fnv1a64_init();
    std::uint64_t h2 = fnv1a64_init();

    hash_string_(h1, h2, polar_id);
    hash_stations(h1, h2, g, q, *this);
    hash_env(h1, h2, e, q, *this);
    hash_op(h1, h2, op, q, *this);
    hash_cfg(h1, h2, cfg, q, *this);

    // forward-specific
    hash_i64_(h1, h2, qd_(V_inplane_mps, q.vel_mps, 0.0));
    hash_i64_(h1, h2, qd_(fcfg.V_axial_mps, q.vel_mps, 0.0));
    hash_i64_(h1, h2, static_cast<std::int64_t>(fcfg.n_psi));
    hash_i64_(h1, h2, qd_(fcfg.min_phi_rad, q.ang_rad, 0.0));
    hash_i64_(h1, h2, qd_(fcfg.max_phi_rad, q.ang_rad, 0.0));
    hash_i64_(h1, h2, static_cast<std::int64_t>(fcfg.max_iter_vi));
    hash_i64_(h1, h2, qd_(fcfg.tol_vi, q.tol, 0.0));
    hash_i64_(h1, h2, qd_(fcfg.relax_vi, q.tol, 0.0));

    EvalCacheKey k;
    k.kind = 1;
    k.h = h1;
    k.h2 = h2;
    return k;
}

} // namespace lift::bemt
