// main.cpp
// Runs a closed-loop FOC simulation of a PMSM motor and saves results to a CSV

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

    // How long to simulate and how small each time step is
    double dt    = 1e-6;   // 1 microsecond per step
    double t_end = 0.20;   // simulate for 200ms

    // ── Motor parameters ──────────────────────────────────────────────────────
    sim::PMSM::Params mp;
    mp.pole_pairs = 5;
    mp.Rs         = 0.138;
    mp.Ld         = 520e-6;
    mp.Lq         = 550e-6;
    mp.psi_f      = 0.051;
    mp.J          = 2.0e-4;
    mp.B          = 2.0e-4;
    mp.T_load     = 0.1;

    sim::PMSM motor(mp);

    // ── Inverter and modulator ────────────────────────────────────────────────
    double vdc = 480.0;
    sim::Inverter       inv(vdc);
    foc::SVPWMModulator mod(static_cast<float>(vdc));

    // ── Target speed ──────────────────────────────────────────────────────────
    double targetRPM   = 5000.0;  // mechanical speed in RPM
    double targetSpeed = targetRPM * 2.0 * foc::PI / 60.0;  // convert to rad/s

    // ── Speed controller ──────────────────────────────────────────────────────
    foc::SpeedController::Params sc;
    sc.kp     = 0.3f;
    sc.ki     = 0.025f;
    sc.iq_max = 20.0f;

    foc::SpeedController speedCtrl(sc);

    // ── Current controller ────────────────────────────────────────────────────
    foc::CurrentController::Params cc;
    cc.kp_d  = 0.8f;
    cc.ki_d  = 300.0f;
    cc.kp_q  = 0.8f;
    cc.ki_q  = 300.0f;
    cc.Ld    = static_cast<float>(mp.Ld);
    cc.Lq    = static_cast<float>(mp.Lq);
    cc.psi_f = static_cast<float>(mp.psi_f);
    cc.v_max = static_cast<float>(vdc / std::sqrt(3.0));

    foc::CurrentController currentCtrl(cc);

    // ── Set up CSV logging ────────────────────────────────────────────────────
    std::filesystem::create_directories("results");
    std::ofstream csv("results/sim_data.csv");
    if (!csv) { std::cerr << "Failed to open results/sim_data.csv\n"; return 1; }

    csv << "t,rpm_ref,rpm,theta_e_true,"
           "va,vb,vc,duty_a,duty_b,duty_c,"
           "ia,ib,ic,ialpha,ibeta,"
           "id_ref,id_true,iq_ref,iq_true,Te\n"
        << std::fixed << std::setprecision(8);

    // Only log every 20 steps to keep the file a manageable size
    int logEvery = 20;
    int logCount = 0;

    // ── Main simulation loop ──────────────────────────────────────────────────
    for (double t = 0.0; t <= t_end; t += dt) {

        // Ramp the speed target up over the first 100ms, then hold it steady
        double speedRef = (t < 0.10)
            ? targetSpeed * (t / 0.10)
            : targetSpeed;

        // Read the current motor state
        float theta_e = static_cast<float>(motor.theta_e());
        float omega_m = static_cast<float>(motor.omega_m());
        float omega_e = static_cast<float>(mp.pole_pairs * motor.omega_m());

        // Get alpha/beta currents from the motor
        double ia_d, ib_d;
        motor.currents_alphabeta(ia_d, ib_d);
        float i_alpha = static_cast<float>(ia_d);
        float i_beta  = static_cast<float>(ib_d);

        // Rotate alpha/beta currents into the d/q frame using the rotor angle
        float id_meas, iq_meas;
        foc::park(i_alpha, i_beta, theta_e, id_meas, iq_meas);

        // Speed controller outputs iq_ref, id_ref is always zero
        float id_ref = 0.0f;
        float iq_ref = speedCtrl.step(static_cast<float>(speedRef), omega_m, static_cast<float>(dt));

        // Current controller outputs the d/q voltages to apply
        float vd, vq;
        currentCtrl.step(id_ref, iq_ref, id_meas, iq_meas, omega_e, static_cast<float>(dt), vd, vq);

        // Rotate d/q voltages back into alpha/beta
        float v_alpha_ref, v_beta_ref;
        foc::inv_park(vd, vq, theta_e, v_alpha_ref, v_beta_ref);

        // Convert alpha/beta voltages → 3-phase → duty cycles → actual phase voltages
        float va_ref, vb_ref, vc_ref;
        foc::inv_clarke(v_alpha_ref, v_beta_ref, va_ref, vb_ref, vc_ref);

        float duty_a, duty_b, duty_c;
        mod.step(va_ref, vb_ref, vc_ref, duty_a, duty_b, duty_c);

        double va, vb, vc;
        inv.step(static_cast<double>(duty_a), static_cast<double>(duty_b), static_cast<double>(duty_c), va, vb, vc);

        // Convert the applied phase voltages back to alpha/beta to feed the motor model
        float v_alpha_m, v_beta_m;
        foc::clarke(static_cast<float>(va), static_cast<float>(vb), v_alpha_m, v_beta_m);

        // Step the motor simulation forward
        motor.step(dt, static_cast<double>(v_alpha_m), static_cast<double>(v_beta_m));

        // Only write to CSV every logEvery steps
        if (++logCount < logEvery) continue;
        logCount = 0;

        // Reconstruct the 3-phase currents from alpha/beta for logging
        double ia =  ia_d;
        double ib = -0.5 * ia_d + 0.5 * std::sqrt(3.0) * ib_d;
        double ic = -0.5 * ia_d - 0.5 * std::sqrt(3.0) * ib_d;

        csv << t               << ","
            << speedRef * 60.0 / (2.0 * foc::PI)   << ","  
            << motor.omega_m() * 60.0 / (2.0 * foc::PI) << ","  
            << motor.theta_e() << ","
            << va              << "," << vb     << "," << vc     << ","
            << duty_a          << "," << duty_b << "," << duty_c << ","
            << ia              << "," << ib     << "," << ic     << ","
            << ia_d            << "," << ib_d   << ","
            << id_ref          << "," << motor.id() << ","
            << iq_ref          << "," << motor.iq() << ","
            << motor.torque()  << "\n";
    }

    std::cout << "Done. Wrote results/sim_data.csv\n";
    return 0;
}