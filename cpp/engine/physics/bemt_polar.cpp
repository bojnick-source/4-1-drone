#include "engine/physics/bemt_polar.hpp"

#include <cmath>

namespace lift::bemt {

void TabularPolar::Slice::validate() const {
    LIFT_BEMT_REQUIRE(alpha_rad.size() >= 5, ErrorCode::InvalidInput, "Polar slice alpha too small");
    LIFT_BEMT_REQUIRE(alpha_rad.size() == cl.size(), ErrorCode::InvalidInput, "Polar slice cl size mismatch");
    LIFT_BEMT_REQUIRE(alpha_rad.size() == cd.size(), ErrorCode::InvalidInput, "Polar slice cd size mismatch");

    double prev = -1e300;
    for (std::size_t i = 0; i < alpha_rad.size(); ++i) {
        LIFT_BEMT_REQUIRE(is_finite(alpha_rad[i]), ErrorCode::InvalidInput, "Polar slice alpha non-finite");
        LIFT_BEMT_REQUIRE(is_finite(cl[i]) && is_finite(cd[i]), ErrorCode::InvalidInput, "Polar slice cl/cd non-finite");
        LIFT_BEMT_REQUIRE(alpha_rad[i] > prev, ErrorCode::InvalidInput, "Polar slice alpha not strictly increasing");
        prev = alpha_rad[i];
        LIFT_BEMT_REQUIRE(cd[i] >= 0.0, ErrorCode::InvalidInput, "Polar slice cd negative");
    }
}

void TabularPolar::add_slice(double reynolds, double mach, Slice s) {
    LIFT_BEMT_REQUIRE(!finalized_, ErrorCode::InvalidInput, "TabularPolar already finalized");
    LIFT_BEMT_REQUIRE(is_finite(reynolds) && reynolds > 0.0, ErrorCode::InvalidInput, "add_slice: reynolds invalid");
    LIFT_BEMT_REQUIRE(is_finite(mach) && mach >= 0.0, ErrorCode::InvalidInput, "add_slice: mach invalid");
    s.validate();
    auto [it, inserted] = slices_.insert({SliceKey{reynolds, mach}, std::move(s)});
    LIFT_BEMT_REQUIRE(inserted, ErrorCode::InvalidInput, "TabularPolar duplicate slice key");
}

void TabularPolar::finalize() {
    LIFT_BEMT_REQUIRE(!slices_.empty(), ErrorCode::InvalidInput, "TabularPolar has no slices");
    finalized_ = true;
}

PolarOutput TabularPolar::sample_in_slice_(const Slice& s, double alpha_rad_q) const {
    // Clamp alpha to slice range (safe for optimizer and avoids throwing on minor excursions)
    const double a0 = s.alpha_rad.front();
    const double a1 = s.alpha_rad.back();
    const double a  = clamp(alpha_rad_q, a0, a1);

    // Find upper bound
    auto it = std::upper_bound(s.alpha_rad.begin(), s.alpha_rad.end(), a);
    if (it == s.alpha_rad.begin()) {
        return PolarOutput{s.cl.front(), s.cd.front()};
    }
    if (it == s.alpha_rad.end()) {
        return PolarOutput{s.cl.back(), s.cd.back()};
    }

    const std::size_t j1 = static_cast<std::size_t>(std::distance(s.alpha_rad.begin(), it));
    const std::size_t j0 = j1 - 1;

    const double x0 = s.alpha_rad[j0];
    const double x1v = s.alpha_rad[j1];
    const double t = safe_div(a - x0, x1v - x0);

    const double clv = lerp_(s.cl[j0], s.cl[j1], t);
    const double cdv = lerp_(s.cd[j0], s.cd[j1], t);

    return PolarOutput{clv, cdv};
}

std::vector<std::pair<TabularPolar::SliceKey, const TabularPolar::Slice*>>
TabularPolar::nearest_slices_(double reynolds, double mach) const {
    // Strategy:
    // - if exactly one slice: return it
    // - otherwise: pick up to 4 slices that bracket Re and Mach (nearest in each dimension),
    //   then allow bilinear weighting
    std::vector<std::pair<SliceKey, const Slice*>> out;
    if (slices_.size() == 1) {
        const auto& kv = *slices_.begin();
        out.push_back({kv.first, &kv.second});
        return out;
    }

    // Collect unique Re and Mach grid values
    std::vector<double> res;
    std::vector<double> mas;
    res.reserve(slices_.size());
    mas.reserve(slices_.size());
    for (const auto& kv : slices_) {
        res.push_back(kv.first.reynolds);
        mas.push_back(kv.first.mach);
    }
    std::sort(res.begin(), res.end());
    res.erase(std::unique(res.begin(), res.end()), res.end());
    std::sort(mas.begin(), mas.end());
    mas.erase(std::unique(mas.begin(), mas.end()), mas.end());

    auto bracket = [](const std::vector<double>& xs, double q) -> std::pair<double,double> {
        if (xs.empty()) return {q, q};
        if (q <= xs.front()) return {xs.front(), xs.front()};
        if (q >= xs.back()) return {xs.back(), xs.back()};
        auto it = std::upper_bound(xs.begin(), xs.end(), q);
        const double hi = *it;
        const double lo = *(it - 1);
        return {lo, hi};
    };

    const auto [re0, re1] = bracket(res, reynolds);
    const auto [m0,  m1 ] = bracket(mas, mach);

    // Probe up to 4 corner keys (some may be missing if sparse grid)
    const SliceKey keys[4] = {
        {re0, m0}, {re1, m0}, {re0, m1}, {re1, m1}
    };
    for (const auto& k : keys) {
        auto it = slices_.find(k);
        if (it != slices_.end()) {
            out.push_back({it->first, &it->second});
        }
    }

    // Fallback: if grid sparse, use nearest slice in L2 sense
    if (out.empty()) {
        double best = std::numeric_limits<double>::infinity();
        const Slice* bestS = nullptr;
        SliceKey bestK{};
        for (const auto& kv : slices_) {
            const double dre = (kv.first.reynolds - reynolds) / std::max(1.0, reynolds);
            const double dm  = (kv.first.mach - mach);
            const double d2  = dre*dre + dm*dm;
            if (d2 < best) { best = d2; bestS = &kv.second; bestK = kv.first; }
        }
        if (bestS) out.push_back({bestK, bestS});
    }

    return out;
}

PolarOutput TabularPolar::sample(const PolarQuery& q) const {
    LIFT_BEMT_REQUIRE(finalized_, ErrorCode::InvalidInput, "TabularPolar not finalized");
    LIFT_BEMT_REQUIRE(is_finite(q.aoa_rad), ErrorCode::InvalidInput, "PolarQuery.alpha non-finite");
    LIFT_BEMT_REQUIRE(is_finite(q.reynolds) && q.reynolds > 0.0, ErrorCode::InvalidInput, "PolarQuery.Re invalid");
    LIFT_BEMT_REQUIRE(is_finite(q.mach) && q.mach >= 0.0, ErrorCode::InvalidInput, "PolarQuery.Mach invalid");

    const auto slices = nearest_slices_(q.reynolds, q.mach);
    LIFT_BEMT_REQUIRE(!slices.empty(), ErrorCode::InvalidInput, "TabularPolar has no usable slices");

    if (slices.size() == 1) {
        return sample_in_slice_(*slices[0].second, q.aoa_rad);
    }

    // If multiple corners exist, do weighted blend.
    // Weights based on normalized distance in Re and Mach using bracketing keys.
    // For sparse cases (2-3 slices), blend by inverse-distance.
    if (slices.size() < 4) {
        double wsum = 0.0;
        double clsum = 0.0, cdsum = 0.0;
        for (const auto& kv : slices) {
            const double dre = (kv.first.reynolds - q.reynolds) / std::max(1.0, q.reynolds);
            const double dm  = (kv.first.mach - q.mach);
            const double d2  = dre*dre + dm*dm;
            const double w   = 1.0 / std::max(1e-12, d2);
            const auto po = sample_in_slice_(*kv.second, q.aoa_rad);
            clsum += w * po.cl;
            cdsum += w * po.cd;
            wsum  += w;
        }
        return PolarOutput{clsum / wsum, cdsum / wsum};
    }

    // 4-corner bilinear (assumes corners exist)
    // Determine bracketing values from corners
    double re0 = slices[0].first.reynolds, re1 = slices[0].first.reynolds;
    double m0 = slices[0].first.mach, m1 = slices[0].first.mach;
    for (const auto& kv : slices) {
        re0 = std::min(re0, kv.first.reynolds);
        re1 = std::max(re1, kv.first.reynolds);
        m0  = std::min(m0,  kv.first.mach);
        m1  = std::max(m1,  kv.first.mach);
    }
    const double tre = (re1 == re0) ? 0.0 : safe_div(q.reynolds - re0, re1 - re0);
    const double tm  = (m1  == m0 ) ? 0.0 : safe_div(q.mach - m0,     m1 - m0);

    auto find_slice = [&](double re, double ma) -> const Slice& {
        auto it = slices_.find(SliceKey{re, ma});
        if (it == slices_.end()) throw BemtError(ErrorCode::InvalidInput, "Missing bilinear polar corner slice");
        return it->second;
    };

    const auto& S00 = find_slice(re0, m0);
    const auto& S10 = find_slice(re1, m0);
    const auto& S01 = find_slice(re0, m1);
    const auto& S11 = find_slice(re1, m1);

    const auto P00 = sample_in_slice_(S00, q.aoa_rad);
    const auto P10 = sample_in_slice_(S10, q.aoa_rad);
    const auto P01 = sample_in_slice_(S01, q.aoa_rad);
    const auto P11 = sample_in_slice_(S11, q.aoa_rad);

    const double cl0 = lerp_(P00.cl, P10.cl, tre);
    const double cl1 = lerp_(P01.cl, P11.cl, tre);
    const double cd0 = lerp_(P00.cd, P10.cd, tre);
    const double cd1 = lerp_(P01.cd, P11.cd, tre);

    return PolarOutput{lerp_(cl0, cl1, tm), lerp_(cd0, cd1, tm)};
}

} // namespace lift::bemt
