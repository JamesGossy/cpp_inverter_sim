// pmsm.hpp
// Simulates a permanent magnet synchronous motor (PMSM).

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

    // Advance the motor by one time step given alpha/beta voltages
    void step(double dt, double v_alpha, double v_beta);

    double theta_e() const { return std::fmod(params.pole_pairs * theta_m, TWO_PI); }
    double omega_m() const { return speed; }
    double torque() const { return Te; }
    double id() const { return id_; }
    double iq() const { return iq_; }

    // Return the stator currents in the alpha/beta frame
    void currents_alphabeta(double& i_alpha, double& i_beta) const;
    void currents_alphabeta(float&  i_alpha, float&  i_beta) const {
        double a, b; currents_alphabeta(a, b);
        i_alpha = (float)a; i_beta = (float)b;
    }

private:
    Params params;
    double id_ = 0.0;
    double iq_ = 0.0;
    double speed = 0.0;  // mechanical speed (rad/s)
    double theta_m = 0.0;  // mechanical angle (rad)
    double Te = 0.0;  // electromagnetic torque (N·m)
};

}