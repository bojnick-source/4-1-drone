#include "engine/physics/bemt_adapters.hpp"

#include <algorithm>
#include <cmath>

namespace lift::bemt {

void RotorParam::validate() const {
    LIFT_BEMT_REQUIRE(blade_count >= 2, ErrorCode::InvalidInput, "RotorParam.blade_count < 2");
    LIFT_BEMT_REQUIRE(is_finite(radius_m) && radius_m > 0.0, ErrorCode::InvalidInput, "RotorParam.radius invalid");
    LIFT_BEMT_REQUIRE(is_finite(hub_radius_m) && hub_radius_m >= 0.0 && hub_radius_m < radius_m, ErrorCode::InvalidInput, "RotorParam.hub_radius invalid");

    const std::size_t n = r_over_R.size();
    LIFT_BEMT_REQUIRE(n >= 5, ErrorCode::InvalidInput, "RotorParam requires >= 5 stations");
    LIFT_BEMT_REQUIRE(chord_m.size() == n, ErrorCode::InvalidInput, "RotorParam chord size mismatch");
    LIFT_BEMT_REQUIRE(twist_rad.size() == n, ErrorCode::InvalidInput, "RotorParam twist size mismatch");
    LIFT_BEMT_REQUIRE(airfoil_id.size() == n, ErrorCode::InvalidInput, "RotorParam airfoil_id size mismatch");

    double prev = -1.0;
    const double hubR = hub_radius_m / radius_m;

    for (std::size_t i = 0; i < n; ++i) {
        LIFT_BEMT_REQUIRE(is_finite(r_over_R[i]) && r_over_R[i] > 0.0, ErrorCode::InvalidInput, "RotorParam r/R invalid");
        LIFT_BEMT_REQUIRE(r_over_R[i] > hubR && r_over_R[i] <= 1.0, ErrorCode::InvalidInput, "RotorParam r/R out of bounds");
        LIFT_BEMT_REQUIRE(r_over_R[i] > prev, ErrorCode::InvalidInput, "RotorParam r/R not strictly increasing");
        prev = r_over_R[i];

        LIFT_BEMT_REQUIRE(is_finite(chord_m[i]) && chord_m[i] > 0.0, ErrorCode::InvalidInput, "RotorParam chord invalid");
        LIFT_BEMT_REQUIRE(is_finite(twist_rad[i]), ErrorCode::InvalidInput, "RotorParam twist invalid");
        LIFT_BEMT_REQUIRE(!airfoil_id[i].empty(), ErrorCode::InvalidInput, "RotorParam airfoil_id empty");
    }
}

static double lerp(double a, double b, double t) noexcept { return a + (b - a) * t; }

static double interp1(const std::vector<double>& x, const std::vector<double>& y, double xq) {
    LIFT_BEMT_REQUIRE(x.size() == y.size(), ErrorCode::InvalidInput, "interp1 size mismatch");
    LIFT_BEMT_REQUIRE(x.size() >= 2, ErrorCode::InvalidInput, "interp1 needs >=2 points");
    LIFT_BEMT_REQUIRE(is_finite(xq), ErrorCode::InvalidInput, "interp1 xq non-finite");

    if (xq <= x.front()) return y.front();
    if (xq >= x.back())  return y.back();

    auto it = std::upper_bound(x.begin(), x.end(), xq);
    const std::size_t j1 = static_cast<std::size_t>(std::distance(x.begin(), it));
    const std::size_t j0 = j1 - 1;

    const double x0 = x[j0], x1 = x[j1];
    const double t = safe_div(xq - x0, x1 - x0);
    const double v = lerp(y[j0], y[j1], t);
    LIFT_BEMT_REQUIRE(is_finite(v), ErrorCode::NumericalFailure, "interp1 produced non-finite");
    return v;
}

RotorGeometry build_rotor_geometry(const RotorParam& p, const GeometryBuildOptions& opt) {
    p.validate();
    opt.validate();

    RotorGeometry g;
    g.blade_count = p.blade_count;
    g.radius_m = p.radius_m;
    g.hub_radius_m = p.hub_radius_m;
    g.tip_loss = p.tip_loss;

    const std::size_t n = opt.resample ? opt.resample_N : p.r_over_R.size();
    g.stations.clear();
    g.stations.reserve(n);

    if (!opt.resample) {
        for (std::size_t i = 0; i < p.r_over_R.size(); ++i) {
            BladeStation s;
            s.r_m = p.r_over_R[i] * p.radius_m;
            s.chord_m = p.chord_m[i];
            s.twist_rad = p.twist_rad[i];
            g.stations.push_back(s);
        }
        g.validate();
        return g;
    }

    // Resample uniformly in radius between first r and 1.0R, keeping hub bound
    const double r0 = std::max(p.r_over_R.front(), p.hub_radius_m / p.radius_m + 1e-6);
    const double r1 = 1.0;

    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i + 1) / static_cast<double>(n + 1);
        const double rr = r0 + t * (r1 - r0);

        BladeStation s;
        s.r_m = rr * p.radius_m;
        s.chord_m = interp1(p.r_over_R, p.chord_m, rr);
        s.twist_rad = interp1(p.r_over_R, p.twist_rad, rr);
        g.stations.push_back(s);
    }

    g.validate();
    return g;
}

PiecewisePolar::PiecewisePolar(std::vector<Node> nodes) : nodes_(std::move(nodes)) {
    LIFT_BEMT_REQUIRE(nodes_.size() >= 1, ErrorCode::InvalidInput, "PiecewisePolar requires >=1 node");
    double prev = -1.0;
    for (const auto& n : nodes_) {
        LIFT_BEMT_REQUIRE(is_finite(n.r_m) && n.r_m > 0.0, ErrorCode::InvalidInput, "PiecewisePolar node r invalid");
        LIFT_BEMT_REQUIRE(n.r_m > prev, ErrorCode::InvalidInput, "PiecewisePolar nodes not strictly increasing");
        LIFT_BEMT_REQUIRE(static_cast<bool>(n.polar), ErrorCode::InvalidInput, "PiecewisePolar node polar null");
        prev = n.r_m;
    }
}

PolarOutput PiecewisePolar::sample(const PolarQuery& q) const {
    // Default: mid-node
    const std::size_t mid = nodes_.size() / 2;
    return nodes_[mid].polar->sample(q);
}

PolarOutput PiecewisePolar::sample_at_radius(double r_m, const PolarQuery& q) const {
    LIFT_BEMT_REQUIRE(is_finite(r_m) && r_m > 0.0, ErrorCode::InvalidInput, "PiecewisePolar sample_at_radius r invalid");

    // Nearest-node selection by radius (stable and cheap)
    auto it = std::lower_bound(nodes_.begin(), nodes_.end(), r_m,
        [](const Node& n, double rv) { return n.r_m < rv; });

    if (it == nodes_.begin()) return it->polar->sample(q);
    if (it == nodes_.end())   return nodes_.back().polar->sample(q);

    const auto& hi = *it;
    const auto& lo = *(it - 1);
    const double dlo = std::abs(r_m - lo.r_m);
    const double dhi = std::abs(hi.r_m - r_m);
    const auto* pick = (dlo <= dhi) ? lo.polar.get() : hi.polar.get();
    return pick->sample(q);
}

PolarOutput RadiusAwarePolar::sample(const PolarQuery& q) const {
    // If radius isn't set, fall back to PiecewisePolar default
    if (!is_finite(r_m_)) return pp_.sample(q);
    return pp_.sample_at_radius(r_m_, q);
}

} // namespace lift::bemt
