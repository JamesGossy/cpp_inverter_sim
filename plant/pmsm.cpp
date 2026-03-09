// pmsm.cpp
// PMSM simulation step: updates currents, torque, speed, and position.

#include "pmsm.hpp"
#include <cmath>

namespace sim {

void PMSM::step(double dt, double v_alpha, double v_beta) {

    double theta = theta_e();
    double we = params.pole_pairs * speed;

    // Rotate voltages from α/β into the d/q frame.
    double c = std::cos(theta);
    double s = std::sin(theta);
    double vd =  c * v_alpha + s * v_beta;
    double vq = -s * v_alpha + c * v_beta;

    // Current derivatives from the motor voltage equations.
    double did = (vd - params.Rs * id + we * params.Lq * iq) / params.Ld;
    double diq = (vq - params.Rs * iq - we * (params.Ld * id + params.psi_f)) / params.Lq;

    id += did * dt;
    iq += diq * dt;

    // Electromagnetic torque.
    Te = 1.5 * params.pole_pairs * (params.psi_f * iq + (params.Ld - params.Lq) * id * iq);

    // Speed and position.
    double dspeed = (Te - params.T_load - params.B * speed) / params.J;
    speed += dspeed * dt;
    theta_m += speed * dt;
}

void PMSM::currents_alphabeta(double& i_alpha, double& i_beta) const {
    double theta = theta_e();
    double c = std::cos(theta);
    double s = std::sin(theta);
    i_alpha = c * id - s * iq;
    i_beta  = s * id + c * iq;
}

} 