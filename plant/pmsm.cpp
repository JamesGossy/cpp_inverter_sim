// pmsm.cpp
// PMSM simulation step: updates currents, torque, speed, and position.

#include "pmsm.hpp"
#include <cmath>

namespace sim {

void PMSM::step(double dt, double v_alpha, double v_beta) {

    double theta = theta_e();
    double we = params.pole_pairs * speed;

    // Rotate voltages from alpha/beta into the d/q frame
    double c  = std::cos(theta);
    double s  = std::sin(theta);
    double vd = c * v_alpha + s * v_beta;
    double vq = -s * v_alpha + c * v_beta;

    // Current derivatives from the motor voltage equations
    double did = (vd - params.Rs * id_ + we * params.Lq * iq_) / params.Ld;
    double diq = (vq - params.Rs * iq_ - we * (params.Ld * id_ + params.psi_f)) / params.Lq;

    id_ += did * dt;
    iq_ += diq * dt;

    // Electromagnetic torque
    Te = 1.5 * params.pole_pairs * (params.psi_f * iq_ + (params.Ld - params.Lq) * id_ * iq_);

    // Speed and position
    double dspeed = (Te - params.T_load - params.B * speed) / params.J;
    speed += dspeed * dt;
    theta_m += speed * dt;
}

void PMSM::currents_alphabeta(double& i_alpha, double& i_beta) const {
    double theta = theta_e();
    double c = std::cos(theta);
    double s = std::sin(theta);
    i_alpha = c * id_ - s * iq_;
    i_beta  = s * id_ + c * iq_;
}

}