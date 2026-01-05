/*
===============================================================================
Fragment 3.1.07 — Airfoil Polar Interface + Linear/Tabulated Models (C++)
File: airfoil_polar.cpp
===============================================================================
*/

#include "engine/physics/airfoil_polar.hpp"

#include <algorithm>
#include <cmath>

namespace lift::bemt {

namespace {
std::size_t upper_index(const std::vector<double>& axis, double x) {
    // returns i such that axis[i] <= x < axis[i+1], clamped to [0, n-2]
    const std::size_t n = axis.size();
    if (n < 2) return 0;

    if (x <= axis.front()) return 0;
    if (x >= axis.back()) return n - 2;

    auto it = std::upper_bound(axis.begin(), axis.end(), x);
    const std::size_t j = static_cast<std::size_t>(std::distance(axis.begin(), it));
    return (j == 0) ? 0 : (j - 1);
}

double lerp(double a, double b, double t) noexcept {
    return a + (b - a) * t;
}

double clamp_or_throw(double x, double lo, double hi, PolarOORPolicy policy) {
    if (x < lo || x > hi) {
        if (policy == PolarOORPolicy::Throw) {
            fail(ErrorCode::PolarOutOfRange, "Polar query out of range", LIFT_BEMT_SITE);
        }
        return clamp(x, lo, hi);
    }
    return x;
}
} // namespace

PolarOutput TabulatedPolar::sample(const PolarQuery& q) const {
    validate();
    q.validate();

    const double aoa_q = clamp_or_throw(q.aoa_rad, aoa_rad.front(), aoa_rad.back(), policy);
    const double re_q  = clamp_or_throw(q.reynolds, reynolds.front(), reynolds.back(), policy);

    const std::size_t Na = aoa_rad.size();
    const std::size_t Nr = reynolds.size();

    const std::size_t ia = upper_index(aoa_rad, aoa_q);
    const std::size_t ir = upper_index(reynolds, re_q);

    const double a0 = aoa_rad[ia];
    const double a1 = aoa_rad[ia + 1];
    const double r0 = reynolds[ir];
    const double r1 = reynolds[ir + 1];

    const double ta = (a1 > a0) ? clamp((aoa_q - a0) / (a1 - a0), 0.0, 1.0) : 0.0;
    const double tr = (r1 > r0) ? clamp((re_q  - r0) / (r1 - r0), 0.0, 1.0) : 0.0;

    const auto idx = [&](std::size_t A, std::size_t R) -> std::size_t {
        return A * Nr + R;
    };

    const double cl00 = cl[idx(ia,     ir)];
    const double cl01 = cl[idx(ia,     ir + 1)];
    const double cl10 = cl[idx(ia + 1, ir)];
    const double cl11 = cl[idx(ia + 1, ir + 1)];

    const double cd00 = cd[idx(ia,     ir)];
    const double cd01 = cd[idx(ia,     ir + 1)];
    const double cd10 = cd[idx(ia + 1, ir)];
    const double cd11 = cd[idx(ia + 1, ir + 1)];

    const double cl0 = lerp(cl00, cl01, tr);
    const double cl1 = lerp(cl10, cl11, tr);
    const double clq = lerp(cl0,  cl1,  ta);

    const double cd0 = lerp(cd00, cd01, tr);
    const double cd1 = lerp(cd10, cd11, tr);
    const double cdq = lerp(cd0,  cd1,  ta);

    PolarOutput out;
    out.cl = is_finite(clq) ? clq : 0.0;
    out.cd = (is_finite(cdq) && cdq >= 0.0) ? cdq : 0.0;
    return out;
}

} // namespace lift::bemt
