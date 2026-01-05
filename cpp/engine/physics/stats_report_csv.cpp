/*
===============================================================================
Fragment 3.1.15 — Uncertainty Report CSV (Moments + Quantiles + CDF Probe)
File: stats_report_csv.cpp
===============================================================================
*/

#include "engine/physics/stats_report_csv.hpp"

#include <iomanip>
#include <sstream>

namespace lift::bemt::stats {

static void emit_metric(std::ostringstream& os, MetricStats& m,
                        double probe_x, bool emit_probe) {
    // Ensure reservoir sort if quantiles/CDF are queried.
    // Reservoir sorts lazily.
    const double p05 = m.reservoir.quantile(0.05);
    const double p50 = m.reservoir.quantile(0.50);
    const double p95 = m.reservoir.quantile(0.95);

    os
      << m.mean()  << "," << m.stddev() << "," << m.minv() << "," << m.maxv() << ","
      << p05 << "," << p50 << "," << p95;

    if (emit_probe) {
        const double Fx = m.reservoir.cdf(probe_x);
        os << "," << Fx;
    }
}

std::string uncertainty_csv_header() {
    // Columns per metric:
    // mean,std,min,max,p05,p50,p95,(optional cdf_probe)
    return
      "seed,cap,"
      "T_mean,T_std,T_min,T_max,T_p05,T_p50,T_p95,T_CDF_probe,"
      "P_mean,P_std,P_min,P_max,P_p05,P_p50,P_p95,P_CDF_probe,"
      "Q_mean,Q_std,Q_min,Q_max,Q_p05,Q_p50,Q_p95,"
      "vi_mean,vi_std,vi_min,vi_max,vi_p05,vi_p50,vi_p95,"
      "FM_mean,FM_std,FM_min,FM_max,FM_p05,FM_p50,FM_p95,"
      "coll_mean,coll_std,coll_min,coll_max,coll_p05,coll_p50,coll_p95\n";
}

std::string uncertainty_csv_row(const UncertaintyReport& r_in,
                                double thrust_probe_N,
                                double power_probe_W) {
    // Copy to allow quantile/CDF queries (which may sort reservoirs)
    UncertaintyReport r = r_in;

    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(8);

    os << r.seed << "," << r.cap << ",";

    emit_metric(os, r.thrust_N, thrust_probe_N, true);
    os << ",";

    emit_metric(os, r.power_W, power_probe_W, true);
    os << ",";

    emit_metric(os, r.torque_Nm, 0.0, false);
    os << ",";

    emit_metric(os, r.vi_mps, 0.0, false);
    os << ",";

    emit_metric(os, r.fm, 0.0, false);
    os << ",";

    emit_metric(os, r.collective_rad, 0.0, false);
    os << "\n";

    return os.str();
}

std::string uncertainty_csv(const UncertaintyReport& r,
                            double thrust_probe_N,
                            double power_probe_W) {
    std::string out;
    out.reserve(2048);
    out.append(uncertainty_csv_header());
    out.append(uncertainty_csv_row(r, thrust_probe_N, power_probe_W));
    return out;
}

} // namespace lift::bemt::stats
