// pmsm_dq.cpp
// PMSM motor model in the dq frame.
// Takes applied phase voltages (alpha/beta) and steps electrical + mechanical state.

#include "pmsm.hpp"
#include <cmath>

namespace sim {

void PMSM::step(double dt_s, double v_alpha, double v_beta) {
    const double theta = theta_e();
    const double we    = p_.pole_pairs * omega_m_;

    // Park transform: alpha/beta -> dq using true rotor angle
    const double co = std::cos(theta), si = std::sin(theta);
    const double vd =  co * v_alpha + si * v_beta;
    const double vq = -si * v_alpha + co * v_beta;

    // Electrical dynamics (IPMSM dq model)
    const double did = (vd - p_.Rs * id_ + we * p_.Lq * iq_) / p_.Ld;
    const double diq = (vq - p_.Rs * iq_ - we * (p_.Ld * id_ + p_.psi_f)) / p_.Lq;
    id_ += did * dt_s;
    iq_ += diq * dt_s;

    // Torque
    Te_ = 1.5 * p_.pole_pairs * (p_.psi_f * iq_ + (p_.Ld - p_.Lq) * id_ * iq_);

    // Mechanical dynamics
    const double domega = (Te_ - p_.T_load - p_.B * omega_m_) / p_.J;
    omega_m_ += domega   * dt_s;
    theta_m_ += omega_m_ * dt_s;
}

void PMSM::currents_alphabeta(double& i_alpha, double& i_beta) const {
    const double theta = theta_e();
    const double co = std::cos(theta), si = std::sin(theta);
    i_alpha = co * id_ - si * iq_;
    i_beta  = si * id_ + co * iq_;
}

}