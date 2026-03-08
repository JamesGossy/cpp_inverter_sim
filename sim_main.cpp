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
#include "control/foc_controller.hpp"
#include "sim/inverter.hpp"
#include "sim/logger.hpp"
#include "sim/pmsm.hpp"

namespace sim_cfg {
    constexpr double DT         = 1e-6;
    constexpr double T_END      = 0.20;
    constexpr double TARGET_RPM = 14000.0;
    constexpr int    LOG_EVERY  = 20;
}

int main() {

    namespace mot = cobalt::motor;
    namespace inv = cobalt::inverter;
    namespace g   = cobalt::gains;

    // ── Plant ─────────────────────────────────────────────────────────────
    sim::PMSM     motor({ mot::pole_pairs, mot::Rs, mot::Ld, mot::Lq, mot::psi_f, mot::J, mot::B, mot::T_load });
    sim::Inverter simInv(inv::vdc);

    // ── FOC controller ────────────────────────────────────────────────────
    foc::FocController::Params focParams;
    focParams.speed          = { g::kp_speed, g::ki_speed, g::i_max };
    focParams.fieldWeakening = { g::kp_fw, g::ki_fw, -g::i_max, 0.f, g::fw_voltage_target };
    focParams.current        = { g::kp_d, g::ki_d, g::kp_q, g::ki_q, mot::Ld, mot::Lq, mot::psi_f, inv::v_max };
    focParams.i_max          = g::i_max;

    foc::FocController foc(focParams);

    // ── Simulation bookkeeping ────────────────────────────────────────────
    const double TARGET_SPEED = sim_cfg::TARGET_RPM * 2.0 * foc::PI / 60.0;
    sim::Logger  logger("results/sim_data.csv", sim_cfg::LOG_EVERY);

    // ── Main loop ─────────────────────────────────────────────────────────
    const uint64_t totalTicks = static_cast<uint64_t>(sim_cfg::T_END / sim_cfg::DT) + 1;

    for (uint64_t n = 0; n < totalTicks; ++n) {

        const double t        = static_cast<double>(n) * sim_cfg::DT;
        const double speedRef = (t < 0.1) ? 0.0 : TARGET_SPEED;

        foc::FocController::Measurements meas;
        meas.theta_e = static_cast<float>(motor.theta_e());
        meas.omega_m = static_cast<float>(motor.omega_m());
        meas.omega_e = static_cast<float>(mot::pole_pairs) * meas.omega_m;
        motor.currents_alphabeta(meas.i_alpha, meas.i_beta);
        meas.vdc = inv::vdc;  // on hardware: read from bus voltage ADC each tick

        foc::FocController::References refs;
        refs.speed_ref = static_cast<float>(speedRef);

        // Outer loop – 2 kHz
        if (n % 500 == 0)
            foc.stepOuter(meas, refs, 500e-6f);

        // Inner loop – 20 kHz
        if (n % 50 == 0)
            foc.stepInner(meas, 50e-6f);

        // Plant – 1 MHz
        // Drive the motor from the duty cycles the inner loop last produced.
        const auto& inner = foc.state().inner;
        double va, vb, vc;
        simInv.step(inner.duty_a, inner.duty_b, inner.duty_c, va, vb, vc);

        float v_alpha, v_beta;
        foc::clarke(static_cast<float>(va), static_cast<float>(vb), v_alpha, v_beta);
        motor.step(sim_cfg::DT, static_cast<double>(v_alpha), static_cast<double>(v_beta));

        const auto& outer = foc.state().outer;
        logger.log(motor, t, speedRef, outer.id_ref, outer.iq_ref, inner.duty_a, inner.duty_b, inner.duty_c, va, vb, vc);
    }

    std::cout << "Done. Wrote results/sim_data.csv\n";
    return 0;
}