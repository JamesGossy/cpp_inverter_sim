// main.cpp
// Top-level simulation. Runs an open-loop V/f drive feeding a PMSM model.
// Logs results to results/sim_data.csv for plotting in visualise_data.py.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "control/angle_generator.hpp"
#include "control/modulation.hpp"
#include "control/transforms.hpp"
#include "sim/inverter.hpp"
#include "sim/pmsm.hpp"

int main() {
    // ---- Simulation parameters ----
    const double dt    = 1e-6;   // Time step (s)
    const double t_end = 0.20;   // Total run time (s)

    // ---- Inverter parameters ----
    const double Vdc  = 480.0;   // DC bus voltage (V)
    const double f_sw = 20000.0; // Switching frequency (Hz)

    // Maximum phase voltage achievable with SVPWM: Vdc / √3
    const double v_phase_max = Vdc * foc::kInvSqrt3;

    foc::AngleGenerator  ang;
    foc::SVPWMModulator  mod((float)Vdc);
    sim::inverter inv(Vdc);

    // ---- Motor parameters ----
    sim::PMSM::Params mp;
    mp.pole_pairs = 5;
    mp.Rs         = 0.138;    // Stator resistance (Ω)
    mp.Ld         = 259e-6;  // d-axis inductance (H)
    mp.Lq         = 275e-6;  // q-axis inductance (H)
    mp.psi_f      = 0.051;   // PM flux linkage (Wb)
    mp.J          = 2.0e-4;  // Rotor inertia (kg·m²)
    mp.B          = 2.0e-4;  // Viscous friction (N·m·s)
    mp.T_load     = 0.1;     // Constant load torque (N·m)

    sim::PMSM motor(mp);

    // ---- Open-loop V/f command ----
    // Frequency ramps from 0 → f_e_target over 0.1 s, then holds.
    const double f_e_target = 200.0;  // Target electrical frequency (Hz)

    // ---- Logging ----
    std::filesystem::create_directories("results");
    std::ofstream csv("results/sim_data.csv");
    if (!csv) { std::cerr << "Failed to open results/sim_data.csv\n"; return 1; }

    csv << "t,f_e_cmd_hz,theta_ctrl,theta_e_true,omega_m,"
           "va,vb,vc,duty_a,duty_b,duty_c,"
           "ia,ib,ic,ialpha,ibeta,id_true,iq_true,Te\n"
        << std::fixed << std::setprecision(8);

    const int log_every = 20;  // Write every Nth step to keep CSV size manageable
    int log_count = 0;

    // ---- Main loop ----
    for (double t = 0.0; t <= t_end; t += dt) {

        // Ramp electrical frequency 0 → f_e_target over the first 0.1 s
        const double f_e_cmd   = (t < 0.10) ? f_e_target * (t / 0.10) : f_e_target;
        const double omega_cmd = 2.0 * foc::kPi * f_e_cmd;

        // Scale voltage with frequency (constant V/f) to avoid saturating the motor at low speed
        const double v_mag = v_phase_max; //v_phase_max * (f_e_cmd / f_e_target);

        // Step the open-loop stator angle integrator
        ang.step((float)omega_cmd, (float)dt);
        const float theta = ang.theta();

        // Compute rotating voltage vector in alpha/beta frame
        const float v_alpha_ref = (float)(v_mag * std::cos(theta));
        const float v_beta_ref  = (float)(v_mag * std::sin(theta));

        // alpha/beta -> abc 
        float va_ref, vb_ref, vc_ref;
        foc::inv_clarke(v_alpha_ref, v_beta_ref, va_ref, vb_ref, vc_ref);

        // abc voltage references -> PWM duty cycles
        float duty_a, duty_b, duty_c;
        mod.step(va_ref, vb_ref, vc_ref, duty_a, duty_b, duty_c);

        // Duty cycles -> phase-to-neutral voltages
        double va, vb, vc;
        inv.step((double)duty_a, (double)duty_b, (double)duty_c, va, vb, vc);

        // Phase voltages -> alpha/beta for the motor model
        float v_alpha_f, v_beta_f;
        foc::clarke((float)va, (float)vb, v_alpha_f, v_beta_f);
        const double v_alpha = v_alpha_f;
        const double v_beta  = v_beta_f;

        // Step the motor model
        motor.step(dt, v_alpha, v_beta);

        // --- Logging (every log_every steps) ---
        if (++log_count < log_every) continue;
        log_count = 0;

        double i_alpha, i_beta;
        motor.currents_alphabeta(i_alpha, i_beta);

        // alpha/beta -> abc for logging
        const double ia =  i_alpha;
        const double ib = -0.5 * i_alpha + 0.5 * std::sqrt(3.0) * i_beta;
        const double ic = -0.5 * i_alpha - 0.5 * std::sqrt(3.0) * i_beta;

        csv << t               << ","
            << f_e_cmd         << ","
            << theta           << ","
            << motor.theta_e() << ","
            << motor.omega_m() << ","
            << va              << "," << vb             << "," << vc             << ","
            << duty_a          << "," << duty_b         << "," << duty_c         << ","
            << ia              << "," << ib             << "," << ic             << ","
            << i_alpha         << "," << i_beta         << ","
            << motor.id()      << "," << motor.iq()     << ","
            << motor.torque()  << "\n";
    }

    std::cout << "Done. Wrote results/sim_data.csv\n";
    return 0;
}