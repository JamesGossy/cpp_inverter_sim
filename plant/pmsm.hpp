// pmsm.hpp
// Permanent magnet synchronous motor simulation model.

#pragma once
#include <cmath>

namespace sim {

const double PI = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;

class PMSM {
public:

    struct Params {
        int pole_pairs;
        double Rs, Ld, Lq, psi_f;
        double J, B, T_load;
        Params() = default;
        Params(int pole_pairs, double Rs, double Ld, double Lq, double psi_f, double J, double B, double T_load) : pole_pairs(pole_pairs), Rs(Rs), Ld(Ld), Lq(Lq), psi_f(psi_f), J(J), B(B), T_load(T_load) {}
    };

    PMSM(Params p) : params(p) {}

    // Advance the motor state by one timestep given α/β voltages.
    void step(double dt, double v_alpha, double v_beta);

    double theta_e() const { return std::fmod(params.pole_pairs * theta_m, TWO_PI); }
    double omega_m() const { return speed; }
    double torque()  const { return Te; }
    double get_id()  const { return id; }
    double get_iq()  const { return iq; }

    // Stator currents in the α/β frame.
    void currents_alphabeta(double& i_alpha, double& i_beta) const;
    void currents_alphabeta(float&  i_alpha, float&  i_beta) const {
        double a, b; currents_alphabeta(a, b);
        i_alpha = (float)a; i_beta = (float)b;
    }

    // Phase A and B currents; C is derived as ic = -(ia + ib).
    void currents_ab(double& ia, double& ib) const {
        double i_alpha, i_beta;
        currents_alphabeta(i_alpha, i_beta);
        ia =  i_alpha;
        ib = (-i_alpha + std::sqrt(3.0) * i_beta) * 0.5;
    }

private:
    Params params;
    double id      = 0.0;  // d-axis current (A)
    double iq      = 0.0;  // q-axis current (A)
    double speed   = 0.0;  // mechanical speed (rad/s)
    double theta_m = 0.0;  // mechanical angle (rad)
    double Te      = 0.0;  // electromagnetic torque (N·m)
};

} 