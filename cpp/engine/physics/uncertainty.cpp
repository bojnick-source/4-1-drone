// ============================================================================
// Fragment 3.3.03 — Uncertainty Sampler (Monte Carlo Wrapper for Scalar Params)
// File: uncertainty.cpp
// ============================================================================

#include "uncertainty.hpp"

#include <cmath>
#include <limits>

namespace lift::bemt::prob {

namespace {

double clampd(double x, double lo, double hi) noexcept {
    if (!is_finite(x)) return lo;
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Box-Muller for standard normal
double std_normal(Rng64& rng) noexcept {
    const double u1 = std::max(1e-12, rng.next_u01());
    const double u2 = rng.next_u01();
    const double r = std::sqrt(-2.0 * std::log(u1));
    const double theta = 2.0 * kPi * u2;
    return r * std::cos(theta);
}

} // namespace

DistSpec uniform(double a, double b, double lo, double hi) {
    DistSpec s;
    s.type = DistType::Uniform;
    s.p1 = a; s.p2 = b;
    s.lo = lo; s.hi = hi;
    s.validate();
    return s;
}

DistSpec normal(double mu, double sigma, double lo, double hi) {
    DistSpec s;
    s.type = DistType::Normal;
    s.p1 = mu; s.p2 = sigma;
    s.lo = lo; s.hi = hi;
    s.validate();
    return s;
}

DistSpec lognormal(double mu_ln, double sigma_ln, double lo, double hi) {
    DistSpec s;
    s.type = DistType::LogNormal;
    s.p1 = mu_ln; s.p2 = sigma_ln;
    s.lo = lo; s.hi = hi;
    s.validate();
    return s;
}

SampleSet sample(const std::string& name, const DistSpec& spec_in, const SamplerConfig& cfg_in) {
    SamplerConfig cfg = cfg_in;
    cfg.validate();

    DistSpec spec = spec_in;
    spec.validate();

    SampleSet out;
    out.name = name;
    out.seed = cfg.seed;
    out.x.resize(cfg.n);

    Rng64 rng(cfg.seed);

    if (spec.type == DistType::Uniform) {
        const double a = spec.p1;
        const double b = spec.p2;
        const double w = b - a;
        for (std::size_t i = 0; i < cfg.n; ++i) {
            const double u = rng.next_u01();
            out.x[i] = clampd(a + w * u, spec.lo, spec.hi);
        }
    } else if (spec.type == DistType::Normal) {
        const double mu = spec.p1;
        const double sigma = spec.p2;
        for (std::size_t i = 0; i < cfg.n; ++i) {
            const double z = std_normal(rng);
            out.x[i] = clampd(mu + sigma * z, spec.lo, spec.hi);
        }
    } else { // LogNormal
        const double mu_ln = spec.p1;
        const double sigma_ln = spec.p2;
        for (std::size_t i = 0; i < cfg.n; ++i) {
            const double z = std_normal(rng);
            const double y = std::exp(mu_ln + sigma_ln * z);
            out.x[i] = clampd(y, spec.lo, spec.hi);
        }
    }

    out.validate();
    return out;
}

} // namespace lift::bemt::prob
