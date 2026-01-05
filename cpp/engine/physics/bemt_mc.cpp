// ============================================================================
// Fragment 3.3.06 — Monte Carlo BEMT Wrapper (Run N samples → CDFs + Prob CSV)
// File: bemt_mc.cpp
// ============================================================================

#include "bemt_mc.hpp"
#include "prob_closeout_csv.hpp"

namespace lift::bemt::prob {

namespace {

double pick_or(const std::unordered_map<std::string, DistSpec>& dists,
               const std::string& key,
               Rng64& rng,
               double baseline) {
    auto it = dists.find(key);
    if (it == dists.end()) return baseline;

    DistSpec s = it->second;
    s.validate();

    // One-off sample using Sampler logic (avoid building whole SampleSet for speed)
    const double u = rng.next_u01();

    auto clampd = [](double x, double lo, double hi) noexcept {
        if (!is_finite(x)) return lo;
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    };

    auto std_normal = [&](Rng64& r) noexcept {
        const double u1 = std::max(1e-12, r.next_u01());
        const double u2 = r.next_u01();
        const double rr = std::sqrt(-2.0 * std::log(u1));
        const double th = 2.0 * kPi * u2;
        return rr * std::cos(th);
    };

    if (s.type == DistType::Uniform) {
        const double a = s.p1, b = s.p2;
        return clampd(a + (b - a) * u, s.lo, s.hi);
    }
    if (s.type == DistType::Normal) {
        const double z = std_normal(rng);
        return clampd(s.p1 + s.p2 * z, s.lo, s.hi);
    }
    // LogNormal
    const double z = std_normal(rng);
    const double y = std::exp(s.p1 + s.p2 * z);
    return clampd(y, s.lo, s.hi);
}

} // namespace

McResult run_bemt_monte_carlo(const std::string& case_id,
                              const McInputs& baseline,
                              const McRunner& runner,
                              const McConfig& cfg_in,
                              const std::vector<ProbGate>& gates) {
    McConfig cfg = cfg_in;
    cfg.validate();

    McResult out;
    out.case_id = case_id;
    out.n_requested = cfg.sampler.n;

    out.hover_power_W.reserve(cfg.sampler.n);
    out.hover_FM.reserve(cfg.sampler.n);
    out.hover_thrust_N.reserve(cfg.sampler.n);
    out.thrust_margin_N.reserve(cfg.sampler.n);
    out.power_margin_W.reserve(cfg.sampler.n);

    // Validate gates early
    for (const auto& g : gates) g.validate();

    Rng64 rng(cfg.sampler.seed);

    for (std::size_t i = 0; i < cfg.sampler.n; ++i) {
        McInputs in = baseline;

        in.rho = pick_or(cfg.dists, "rho", rng, baseline.rho);
        in.mu  = pick_or(cfg.dists, "mu",  rng, baseline.mu);

        in.omega_rad_s     = pick_or(cfg.dists, "omega", rng, baseline.omega_rad_s);
        in.collective_rad  = pick_or(cfg.dists, "collective", rng, baseline.collective_rad);

        in.radius_scale = pick_or(cfg.dists, "radius_scale", rng, baseline.radius_scale);
        in.chord_scale  = pick_or(cfg.dists, "chord_scale",  rng, baseline.chord_scale);

        const McOutputs y = runner(in);

        const bool ok = (y.code == ErrorCode::Ok) &&
                        is_finite(y.hover_T_N) && is_finite(y.hover_P_W) && is_finite(y.hover_FM);

        if (!ok) {
            ++out.n_failed;
            if (!cfg.drop_failed_runs) {
                out.hover_thrust_N.push_back(std::numeric_limits<double>::quiet_NaN());
                out.hover_power_W.push_back(std::numeric_limits<double>::quiet_NaN());
                out.hover_FM.push_back(std::numeric_limits<double>::quiet_NaN());
                out.thrust_margin_N.push_back(std::numeric_limits<double>::quiet_NaN());
                out.power_margin_W.push_back(std::numeric_limits<double>::quiet_NaN());
            }
            continue;
        }

        ++out.n_ok;

        out.hover_thrust_N.push_back(y.hover_T_N);
        out.hover_power_W.push_back(y.hover_P_W);
        out.hover_FM.push_back(y.hover_FM);

        const double t_margin = y.hover_T_N - cfg.required_thrust_N;
        const double p_margin = cfg.hover_power_cap_W - y.hover_P_W;

        out.thrust_margin_N.push_back(t_margin);
        out.power_margin_W.push_back(p_margin);
    }

    // Build CDFs
    out.cdfs.clear();
    out.cdfs.emplace_back("hover_thrust_N", EmpiricalCdf(out.hover_thrust_N));
    out.cdfs.emplace_back("hover_power_W",  EmpiricalCdf(out.hover_power_W));
    out.cdfs.emplace_back("hover_FM",       EmpiricalCdf(out.hover_FM));
    out.cdfs.emplace_back("thrust_margin_N", EmpiricalCdf(out.thrust_margin_N));
    out.cdfs.emplace_back("power_margin_W",  EmpiricalCdf(out.power_margin_W));

    // Summaries (set reasonable thresholds for exceedance on margins)
    out.summaries.clear();
    out.summaries.reserve(out.cdfs.size());

    for (const auto& kv : out.cdfs) {
        const auto& name = kv.first;
        const auto& cdf = kv.second;

        if (name == "thrust_margin_N") {
            out.summaries.push_back(summarize(case_id, name, cdf, 0.0));
        } else if (name == "power_margin_W") {
            out.summaries.push_back(summarize(case_id, name, cdf, 0.0));
        } else {
            out.summaries.push_back(summarize(case_id, name, cdf));
        }
    }

    out.prob_summary_csv = prob_csv(out.summaries);

    // Probabilistic gates (optional)
    if (!gates.empty()) {
        out.gate_report = eval_prob_gates(case_id, gates, out.cdfs);
        out.prob_gate_csv = prob_gate_csv(std::vector<ProbGateReport>{out.gate_report});
    }

    out.code = ErrorCode::Ok;
    out.message = "OK";
    return out;
}

} // namespace lift::bemt::prob
