#include "engine/physics/bemt_solver.hpp"
#include "engine/physics/bemt_polar.hpp"

#include <iostream>

using namespace lift::bemt;

// Minimal hardened example that can be removed later.
// Provides a sanity check path without tying into GitHub tooling.
static TabularPolar make_simple_polar() {
    TabularPolar p;

    // One slice at Re=1e6, Mach=0.1
    TabularPolar::Slice s;
    // alpha from -15 to +15 deg
    for (int d = -15; d <= 15; d += 1) {
        const double a = deg2rad(static_cast<double>(d));
        // Toy polar: linear lift slope with soft stall; quadratic drag
        double cl = 2.0 * kPi * a;           // thin airfoil
        cl = clamp(cl, -1.2, 1.2);
        const double cd = 0.01 + 0.02 * cl * cl;
        s.alpha_rad.push_back(a);
        s.cl.push_back(cl);
        s.cd.push_back(cd);
    }
    p.add_slice(1.0e6, 0.1, std::move(s));
    p.finalize();
    return p;
}

int main() {
    try {
        auto polar = make_simple_polar();
        BemtSolver solver(polar);

        RotorGeometry g;
        g.blade_count = 2;
        g.radius_m = 1.25;
        g.hub_radius_m = 0.15;
        g.tip_loss = TipLossModel::Prandtl;

        // Simple linear chord/twist distribution
        const std::size_t N = 25;
        g.stations.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            const double t = static_cast<double>(i + 1) / static_cast<double>(N + 1);
            const double r = g.hub_radius_m + t * (g.radius_m - g.hub_radius_m);
            BladeStation s;
            s.r_m = r;
            s.chord_m = 0.12 - 0.04 * t;             // taper
            s.twist_rad = deg2rad(18.0 - 10.0 * t);  // washout
            g.stations.push_back(s);
        }

        Environment env;
        env.rho = 1.225;
        env.mu  = 1.81e-5;

        OperatingPoint op;
        op.V_inf = 0.0;
        op.omega_rad_s = 260.0; // ~2483 RPM
        op.target_thrust_N = 1200.0; // ~122 kgf equivalent
        op.collective_offset_rad = 0.0;

        SolverConfig cfg;
        cfg.max_iter_inflow = 200;
        cfg.max_iter_trim = 80;
        cfg.tol_inflow = 1e-6;
        cfg.tol_trim_N = 1e-3;

        BemtInputs in;
        in.geom = g;
        in.env = env;
        in.op = op;
        in.cfg = cfg;

        const auto r = solver.solve(in);

        std::cout << "Thrust (N): " << r.thrust_N << "\n";
        std::cout << "Power (W):  " << r.power_W << "\n";
        std::cout << "Torque (Nm):" << r.torque_Nm << "\n";
        std::cout << "FM:         " << r.figure_of_merit << "\n";
        std::cout << "Collective offset (deg): " << rad2deg(r.collective_offset_rad) << "\n";
        std::cout << "Induced velocity (m/s):  " << r.induced_velocity_m_s << "\n";
        std::cout << "Inflow iters:            " << r.inflow_iters << "\n";
        std::cout << "Trim iters:              " << r.trim_iters << "\n";

        return (r.code == ErrorCode::Ok) ? 0 : 2;
    } catch (const BemtError& e) {
        std::cerr << "BEMT ERROR (" << static_cast<std::uint32_t>(e.code()) << "): " << e.what() << "\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "FATAL: " << e.what() << "\n";
        return 3;
    }
}
