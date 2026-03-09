// sim_main.hpp
// Configuration, setup helpers, and per-tick helpers for the FOC simulation.
// sim_main.cpp only needs to include this and call main().

#pragma once

#include <cmath>
#include <cstdint>

#include "config/cobalt_params.hpp"
#include "control/encoder_compensation.hpp"
#include "control/foc_controller.hpp"
#include "sim/encoder_delay.hpp"
#include "plant/inverter.hpp"
#include "sim/logger.hpp"
#include "plant/pmsm.hpp"
#include "system/faults.hpp"

// ── Simulation timing ──────────────────────────────────────────────────────
namespace sim_cfg {
    constexpr double DT = 1e-6;             // plant timestep (1 MHz)
    constexpr double T_END = 0.20;          // simulation end time (s)
    constexpr double TARGET_RPM = 14000.0;  // speed setpoint
    constexpr int LOG_EVERY = 20;           // log one row every N inner ticks

    constexpr int OUTER_STRIDE = 500;       // 1 MHz / 500 = 2 kHz
    constexpr int INNER_STRIDE = 50;        // 1 MHz /  50 = 20 kHz
    constexpr float OUTER_DT = OUTER_STRIDE * static_cast<float>(DT);
    constexpr float INNER_DT = INNER_STRIDE * static_cast<float>(DT);
}

// ── Factory functions ──────────────────────────────────────────────────────

// Constructs the PMSM plant from cobalt_params.
inline sim::PMSM makePMSM()
{
    namespace m = cobalt::motor;
    return sim::PMSM({ m::pole_pairs, m::Rs, m::Ld, m::Lq, m::psi_f, m::J, m::B, m::T_load });
}

// Constructs the FOC controller from cobalt_params.
inline foc::FocController makeFocController()
{
    namespace m = cobalt::motor;
    namespace i = cobalt::inverter;
    namespace g = cobalt::gains;

    foc::FocController::Params p;
    p.speed = { g::kp_speed, g::ki_speed, g::i_max };
    p.fieldWeakening = { g::kp_fw, g::ki_fw, -g::i_max, 0.f, g::fw_voltage_target };
    p.current = { g::kp_d, g::ki_d, g::kp_q, g::ki_q, m::Ld, m::Lq, m::psi_f, i::v_max };
    p.i_max = g::i_max;
    return foc::FocController(p);
}

// ── Per-tick helpers ───────────────────────────────────────────────────────

// Builds the controller measurement struct from the current plant state.
// Pushes the true angle through the delay line and applies encoder compensation.
inline foc::FocController::Measurements buildMeasurements(const sim::PMSM& motor, sim::EncoderDelay& encoderDelay)
{
    foc::FocController::Measurements meas;
    meas.omega_m = static_cast<float>(motor.omega_m());
    meas.omega_e = static_cast<float>(cobalt::motor::pole_pairs) * meas.omega_m;

    const float theta_delayed = encoderDelay.push(static_cast<float>(motor.theta_e()));
    meas.theta_e = foc::compensateEncoderDelay(theta_delayed, meas.omega_e);

    // Phase currents * dc voltage: read A and B directly; C is derived as ic = -(ia + ib).
    double ia, ib;
    motor.currents_ab(ia, ib);
    meas.ia  = static_cast<float>(ia);
    meas.ib  = static_cast<float>(ib);
    meas.vdc = cobalt::inverter::vdc;  
    return meas;
}

// Converts motor state to RPM and peak current, then updates the fault flags.
inline void checkFaults(cobalt::Faults& faults, const sim::PMSM& motor, float vdc)
{
    const float speed_rpm = static_cast<float>(motor.omega_m()) * (60.0f / (2.0f * 3.14159265f));
    const float current_a = std::sqrt(motor.get_id() * motor.get_id() + motor.get_iq() * motor.get_iq());
    cobalt::updateFaults(faults, speed_rpm, vdc, current_a);
}

// Converts duty cycles to phase voltages, then advances the PMSM by one timestep.
inline void stepPlant(sim::Inverter& simInv, sim::PMSM& motor, float duty_a, float duty_b, float duty_c)
{
    double va, vb, vc;
    simInv.step(duty_a, duty_b, duty_c, va, vb, vc);

    // Clarke into α/β before passing to the motor model.
    float v_alpha, v_beta;
    foc::clarke(static_cast<float>(va), static_cast<float>(vb), v_alpha, v_beta);
    motor.step(sim_cfg::DT, v_alpha, v_beta);
}