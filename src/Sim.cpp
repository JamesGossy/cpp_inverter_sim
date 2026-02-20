#include "Sim.hpp"
#include <iostream>
#include <cmath>

namespace olinv {

Sim::Sim(const SimParams& sp, const PMSMParams& mp)
: sp_(sp)
, motor_(mp)
, cc_(CurrentControlParams{mp.R, mp.Ld, mp.Lq, mp.psi_f, sp.Vdc})
, inv_(sp.Vdc)
, spwm_(sp.Vdc)
, logger_(sp.csv_path)
{
    // Default PI gains for plant ~ 1/(Ls+R).
    // Choose current-loop bandwidth ~ 1 kHz (conservative vs 20 kHz switching).
    constexpr double PI = 3.14159265358979323846;
    const double w_cc = 2.0 * PI * 1000.0;

    const double kp_d = mp.Ld * w_cc;
    const double ki_d = mp.R  * w_cc;
    const double kp_q = mp.Lq * w_cc;
    const double ki_q = mp.R  * w_cc;

    cc_.set_pi_gains(kp_d, ki_d, kp_q, ki_q);

    PMSMState x0;
    x0.i_dq = {0.0, 0.0};
    x0.omega_m = 0.0;
    x0.theta_e = 0.0;
    motor_.reset(x0);

    logger_.write_header();
}

Vec2 Sim::current_ref(double t) const {
    // Simple ramp on iq reference (torque command), id held constant
    const double iq = (t < sp_.iq_ref_ramp_time)
        ? sp_.iq_ref_step * (t / sp_.iq_ref_ramp_time)
        : sp_.iq_ref_step;
    return {sp_.id_ref, iq};
}

void Sim::run() {
    const double dt_ctrl = 1.0 / sp_.f_ctrl;
    const double dt_sim  = dt_ctrl / static_cast<double>(sp_.substeps);

    Vec2 v_dq_cmd{0.0, 0.0};
    Vec3 duty_abc{0.5, 0.5, 0.5};
    Vec3 v_abc_ref{0.0, 0.0, 0.0};
    Vec3 v_abc_applied = inv_.phase_voltages_from_duty(duty_abc);

    double t = 0.0;
    int ctrl_countdown = 0;

    const int total_steps = static_cast<int>(std::ceil(sp_.t_end / dt_sim));
    for (int k = 0; k < total_steps; ++k) {
        const Vec2 i_dq = motor_.state().i_dq;
        const double theta_e = motor_.state().theta_e;

        // abc currents for logging (dq -> ab -> abc)
        const Vec2 i_ab = inv_park(i_dq, theta_e);
        const Vec3 i_abc = inv_clarke(i_ab);

        // Control + PWM update at 20 kHz
        if (ctrl_countdown == 0) {
            const Vec2 i_dq_ref = current_ref(t);
            const double omega_e = motor_.omega_e();

            v_dq_cmd = cc_.step(i_dq_ref, i_dq, omega_e, dt_ctrl);

            // dq -> abc voltage reference
            const Vec2 v_ab_ref = inv_park(v_dq_cmd, theta_e);
            v_abc_ref = inv_clarke(v_ab_ref);

            // SPWM duty (average) and average inverter output
            duty_abc = spwm_.duty_from_phase_voltage(v_abc_ref);
            v_abc_applied = inv_.phase_voltages_from_duty(duty_abc);

            // Log once per PWM period
            const double Te = motor_.torque_e();
            logger_.log(t, i_dq_ref, i_dq, i_abc, v_dq_cmd, v_abc_ref, duty_abc,
                        v_abc_applied, Te, motor_.state().omega_m);

            ctrl_countdown = sp_.substeps - 1;
        } else {
            ctrl_countdown--;
        }

        // Applied abc -> dq for motor model integration
        const Vec2 v_ab_applied = clarke(v_abc_applied);
        const Vec2 v_dq_applied = park(v_ab_applied, theta_e);

        motor_.step(v_dq_applied, dt_sim);
        t += dt_sim;
    }

    std::cout << "Simulation complete.\nCSV: " << sp_.csv_path << "\n";
}

} // namespace olinv