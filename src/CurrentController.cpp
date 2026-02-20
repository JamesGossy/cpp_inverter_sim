#include "CurrentController.hpp"

namespace olinv {

CurrentController::CurrentController(const CurrentControlParams& p)
: p_(p)
, pi_d_(0.0, 0.0, PIController::kDefaultLimits)
, pi_q_(0.0, 0.0, PIController::kDefaultLimits)
{}

void CurrentController::reset() {
    pi_d_.reset();
    pi_q_.reset();
}

void CurrentController::set_pi_gains(double kp_d, double ki_d, double kp_q, double ki_q) {
    pi_d_.set_gains(kp_d, ki_d);
    pi_q_.set_gains(kp_q, ki_q);
}

Vec2 CurrentController::limit_vdq_spwm(const Vec2& v_dq) const {
    // Simple magnitude cap: SPWM phase voltage bounded by +/-Vdc/2.
    const double vmax = 0.5 * p_.Vdc;
    const double mag = v_dq.norm();
    if (mag <= vmax || mag <= 1e-12) return v_dq;
    return v_dq * (vmax / mag);
}

Vec2 CurrentController::step(const Vec2& i_dq_ref,
                               const Vec2& i_dq_meas,
                               double omega_e,
                               double dt) {
    const double err_d = i_dq_ref.x - i_dq_meas.x;
    const double err_q = i_dq_ref.y - i_dq_meas.y;

    // Decoupling/feedforward from dq model:
    // v_d = R i_d + Ld di_d/dt - omega_e Lq i_q
    // v_q = R i_q + Lq di_q/dt + omega_e (Ld i_d + psi_f)
    const double v_d_ff = -omega_e * p_.Lq * i_dq_meas.y;
    const double v_q_ff =  omega_e * (p_.Ld * i_dq_meas.x + p_.psi_f);

    const double v_d_pi = pi_d_.update(err_d, dt);
    const double v_q_pi = pi_q_.update(err_q, dt);

    const Vec2 v_dq_cmd_raw{v_d_pi + v_d_ff, v_q_pi + v_q_ff};
    return limit_vdq_spwm(v_dq_cmd_raw);
}

} // namespace olinv