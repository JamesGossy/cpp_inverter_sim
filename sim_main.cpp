// sim_main.cpp
// Closed-loop FOC simulation of a PMSM.
//
//  Outer loop  –   2 kHz  (every 500 ticks) : field-weakening + speed controller
//  Inner loop  –  20 kHz  (every  50 ticks) : transforms, current controllers, SVPWM
//  Plant       –   1 MHz  (every tick)      : inverter + PMSM integration

#include <cmath>
#include <cstdint>
#include <iostream>

#include "config/cobalt_params.hpp"
#include "control/current_controller.hpp"
#include "control/field_weakening_controller.hpp"
#include "control/modulation.hpp"
#include "control/speed_controller.hpp"
#include "control/transforms.hpp"
#include "sim/inverter.hpp"
#include "sim/logger.hpp"
#include "sim/pmsm.hpp"

namespace sim_cfg {
    constexpr double DT = 1e-6;
    constexpr double T_END = 0.20;
    constexpr double TARGET_RPM = 20000.0;
    constexpr int LOG_EVERY = 20;
}

int main() {

    namespace mot = cobalt::motor;
    namespace inv = cobalt::inverter;
    namespace g = cobalt::gains;

    // ── Setup ─────────────────────────────────────────────────────────────
    sim::PMSM motor({ mot::pole_pairs, mot::Rs, mot::Ld, mot::Lq, mot::psi_f, mot::J,  mot::B,  mot::T_load });

    sim::Inverter simInv(inv::vdc);
    foc::SVPWMModulator mod(inv::vdc);

    foc::SpeedController speedCtrl({ g::kp_speed, g::ki_speed, g::i_max });
    foc::FieldWeakeningController fwCtrl({ 0.f, g::ki_fw, -g::i_max, 0.f, g::fw_voltage_target });
    foc::CurrentController currentCtrl({ g::kp_d, g::ki_d, g::kp_q, g::ki_q, mot::Ld, mot::Lq, mot::psi_f, inv::v_max });

    // ── Shared state between tiers ────────────────────────────────────────
    float theta_e = 0.f, omega_m = 0.f, omega_e = 0.f;
    float i_alpha = 0.f, i_beta = 0.f;
    float id_ref = 0.f, iq_ref = 0.f;
    float duty_a = 0.5f, duty_b = 0.5f, duty_c = 0.5f;
    float vMag = 0.f;

    const double TARGET_SPEED = sim_cfg::TARGET_RPM * 2.0 * foc::PI / 60.0;
    sim::Logger logger("results/sim_data.csv", sim_cfg::LOG_EVERY);

    // ── Main loop ─────────────────────────────────────────────────────────
    const uint64_t totalTicks = static_cast<uint64_t>(sim_cfg::T_END / sim_cfg::DT) + 1;

    for (uint64_t n = 0; n < totalTicks; ++n) {

        const double t = static_cast<double>(n) * sim_cfg::DT;
        const double speedRef = (t < 0.1) ? 0.0 : TARGET_SPEED;

        // Outer loop – 2 kHz
        if (n % 500 == 0) {
            id_ref = fwCtrl.step(vMag, inv::v_max, 500e-6f);
            float id_sat = std::clamp(id_ref, -g::i_max, g::i_max);
            float iq_limit = std::sqrt(std::max(0.f, g::i_max * g::i_max - id_sat * id_sat));
            iq_ref = speedCtrl.step(static_cast<float>(speedRef), omega_m, 500e-6f, iq_limit);
        }

        // Inner loop – 20 kHz
        if (n % 50 == 0) {
            float id_meas, iq_meas;
            foc::park(i_alpha, i_beta, theta_e, id_meas, iq_meas);

            float vd, vq;
            currentCtrl.step(id_ref, iq_ref, id_meas, iq_meas, omega_e, 50e-6f, vd, vq);
            vMag = std::sqrt(vd * vd + vq * vq);

            float v_alpha, v_beta, va, vb, vc;
            foc::inv_park(vd, vq, theta_e, v_alpha, v_beta);
            foc::inv_clarke(v_alpha, v_beta, va, vb, vc);
            mod.step(va, vb, vc, duty_a, duty_b, duty_c);
        }

        // Plant – 1 MHz
        double va, vb, vc;
        simInv.step(duty_a, duty_b, duty_c, va, vb, vc);

        float v_alpha, v_beta;
        foc::clarke((float)va, (float)vb, v_alpha, v_beta);
        motor.step(sim_cfg::DT, (double)v_alpha, (double)v_beta);

        theta_e = (float)motor.theta_e();
        omega_m = (float)motor.omega_m();
        omega_e = (float)mot::pole_pairs * omega_m;
        motor.currents_alphabeta(i_alpha, i_beta);

        logger.log(motor, t, speedRef, id_ref, iq_ref, duty_a, duty_b, duty_c, va, vb, vc);
    }

    std::cout << "Done. Wrote results/sim_data.csv\n";
    return 0;
}