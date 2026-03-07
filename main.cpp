// main.cpp
// Closed-loop FOC simulation feeding a PMSM model.
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "control/current_controller.hpp"
#include "control/modulation.hpp"
#include "control/speed_controller.hpp"
#include "control/transforms.hpp"
#include "sim/inverter.hpp"
#include "sim/pmsm.hpp"

int main() {
    // ── Simulation parameters ────────────────────────────────────────────────
    constexpr double dt    = 1e-6;
    constexpr double t_end = 0.20;

    // ── Computational delay ──────────────────────────────────────────────────
    // Models the one PWM period delay between measurement and voltage application
    // Set to 0 to disable.
    constexpr int kDelaySteps = 50;  // 50 µs = 1 switching period at 20 kHz

    std::array<float, kDelaySteps + 1> delay_alpha{}, delay_beta{};
    int delay_head = 0;

    // ── Motor parameters ─────────────────────────────────────────────────────
    sim::PMSM::Params mp;
    mp.pole_pairs = 5;
    mp.Rs         = 0.138;
    mp.Ld         = 259e-6;
    mp.Lq         = 275e-6;
    mp.psi_f      = 0.051;
    mp.J          = 2.0e-4;
    mp.B          = 2.0e-4;
    mp.T_load     = 0.1;

    sim::PMSM motor(mp);

    // ── Inverter ─────────────────────────────────────────────────────────────
    constexpr double Vdc = 480.0;
    sim::inverter       inv(Vdc);
    foc::SVPWMModulator mod(static_cast<float>(Vdc));

    // ── Speed reference profile ──────────────────────────────────────────────
    constexpr double f_e_target     = 200.0;
    const     double omega_m_target = 2.0 * foc::kPi * f_e_target / mp.pole_pairs;

    // ── MTPA current angle ───────────────────────────────────────────────────
    // Advances the current vector off the q-axis into the -d direction.
    // id_ref = -|i| * sin(delta),   iq_ref = |i| * cos(delta)
    constexpr float kMtpaDeg      = 0.0f;
    constexpr float kMtpaRad      = kMtpaDeg * static_cast<float>(foc::kPi) / 180.0f;
    const     float kMtpaSinDelta = std::sin(kMtpaRad);
    const     float kMtpaCosDelta = std::cos(kMtpaRad);

    // ── Speed controller ─────────────────────────────────────────────────────
    // iq_max is the total current vector magnitude limit (A).
    foc::SpeedController::Params sc;
    sc.kp     = 0.3f;
    sc.ki     = 0.025f;
    sc.iq_max = 20.0f;

    foc::SpeedController speed_ctrl(sc);

    // ── Current controller ───────────────────────────────────────────────────
    foc::CurrentController::Params cc;
    cc.kp_d  = 0.5f;
    cc.ki_d  = 300.0f;
    cc.kp_q  = 0.5f;
    cc.ki_q  = 300.0f;
    cc.Ld    = static_cast<float>(mp.Ld);
    cc.Lq    = static_cast<float>(mp.Lq);
    cc.psi_f = static_cast<float>(mp.psi_f);
    cc.v_max = static_cast<float>(Vdc * foc::kInvSqrt3);

    foc::CurrentController current_ctrl(cc);

    // ── Logging ──────────────────────────────────────────────────────────────
    std::filesystem::create_directories("results");
    std::ofstream csv("results/sim_data.csv");
    if (!csv) { std::cerr << "Failed to open results/sim_data.csv\n"; return 1; }

    csv << "t,omega_m_ref,omega_m,theta_e_true,"
           "va,vb,vc,duty_a,duty_b,duty_c,"
           "ia,ib,ic,ialpha,ibeta,"
           "id_ref,id_true,iq_ref,iq_true,Te\n"
        << std::fixed << std::setprecision(8);

    constexpr int log_every = 20;
    int log_count = 0;

    // ── Main loop ─────────────────────────────────────────────────────────────
    for (double t = 0.0; t <= t_end; t += dt) {

        const double omega_m_ref = (t < 0.10)
            ? omega_m_target * (t / 0.10)
            : omega_m_target;

        // Feedback
        const float theta_e = static_cast<float>(motor.theta_e());
        const float omega_m = static_cast<float>(motor.omega_m());
        const float omega_e = static_cast<float>(mp.pole_pairs * motor.omega_m());

        double ia_d, ib_d;
        motor.currents_alphabeta(ia_d, ib_d);
        const float i_alpha = static_cast<float>(ia_d);
        const float i_beta  = static_cast<float>(ib_d);

        // Park transform → dq currents
        float id_meas, iq_meas;
        foc::park(i_alpha, i_beta, theta_e, id_meas, iq_meas);

        // Speed loop → current magnitude → MTPA decomposition into id/iq refs
        const float i_mag  = speed_ctrl.step(static_cast<float>(omega_m_ref), omega_m, static_cast<float>(dt));
        const float id_ref = -i_mag * kMtpaSinDelta;
        const float iq_ref =  i_mag * kMtpaCosDelta;

        // Current loops → dq voltages
        float vd, vq;
        current_ctrl.step(id_ref, iq_ref, id_meas, iq_meas, omega_e, static_cast<float>(dt), vd, vq);

        // Inverse Park → alpha/beta voltage references
        float v_alpha_ref, v_beta_ref;
        foc::inv_park(vd, vq, theta_e, v_alpha_ref, v_beta_ref);

        // ── Computational delay ───────────────────────────────────────────────
        float v_alpha_delayed, v_beta_delayed;
        if constexpr (kDelaySteps > 0) {
            delay_alpha[delay_head] = v_alpha_ref;
            delay_beta [delay_head] = v_beta_ref;
            const int tail = (delay_head + 1) % (kDelaySteps + 1);
            v_alpha_delayed = delay_alpha[tail];
            v_beta_delayed  = delay_beta [tail];
            delay_head = tail;
        } else {
            v_alpha_delayed = v_alpha_ref;
            v_beta_delayed  = v_beta_ref;
        }

        // alpha/beta → abc → duty cycles → phase voltages
        float va_ref, vb_ref, vc_ref;
        foc::inv_clarke(v_alpha_delayed, v_beta_delayed, va_ref, vb_ref, vc_ref);

        float duty_a, duty_b, duty_c;
        mod.step(va_ref, vb_ref, vc_ref, duty_a, duty_b, duty_c);

        double va, vb, vc;
        inv.step(static_cast<double>(duty_a),
                 static_cast<double>(duty_b),
                 static_cast<double>(duty_c), va, vb, vc);

        float v_alpha_m, v_beta_m;
        foc::clarke(static_cast<float>(va), static_cast<float>(vb), v_alpha_m, v_beta_m);

        motor.step(dt, static_cast<double>(v_alpha_m), static_cast<double>(v_beta_m));

        // Logging
        if (++log_count < log_every) continue;
        log_count = 0;

        const double ia =  ia_d;
        const double ib = -0.5 * ia_d + 0.5 * std::sqrt(3.0) * ib_d;
        const double ic = -0.5 * ia_d - 0.5 * std::sqrt(3.0) * ib_d;

        csv << t                << ","
            << omega_m_ref      << ","
            << motor.omega_m()  << ","
            << motor.theta_e()  << ","
            << va               << "," << vb     << "," << vc     << ","
            << duty_a           << "," << duty_b << "," << duty_c << ","
            << ia               << "," << ib     << "," << ic     << ","
            << ia_d             << "," << ib_d   << ","
            << id_ref           << "," << motor.id() << ","
            << iq_ref           << "," << motor.iq() << ","
            << motor.torque()   << "\n";
    }

    std::cout << "Done. Wrote results/sim_data.csv\n";
    return 0;
}