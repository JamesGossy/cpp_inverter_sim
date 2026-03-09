// sim_helpers.hpp
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
#include "logger.hpp"
#include "plant/pmsm.hpp"
#include "system/faults.hpp"

namespace sim_cfg {
    constexpr double DT         = 1e-6;
    constexpr double T_END      = 0.30;
    constexpr double TARGET_RPM = 20000.0;
    constexpr int    LOG_EVERY  = 20;

    constexpr int   OUTER_STRIDE = 500;   // 1 MHz / 500 = 2 kHz
    constexpr int   INNER_STRIDE = 50;    // 1 MHz /  50 = 20 kHz
    constexpr float OUTER_DT     = OUTER_STRIDE * static_cast<float>(DT);
    constexpr float INNER_DT     = INNER_STRIDE * static_cast<float>(DT);
}

// ── Factory functions ──────────────────────────────────────────────────────

inline sim::PMSM makePMSM()
{
    namespace m = cobalt::motor;
    return sim::PMSM({ m::pole_pairs, m::Rs, m::Ld, m::Lq,
                       m::psi_f, m::J, m::B, m::T_load });
}

inline foc::FocController makeFocController()
{
    namespace m = cobalt::motor;
    namespace i = cobalt::inverter;
    namespace g = cobalt::gains;

    foc::FocController::Params p;
    p.speed          = { g::kp_speed, g::ki_speed, g::i_max };
    p.fieldWeakening = { g::kp_fw, g::ki_fw, -g::i_max, 0.f, g::fw_voltage_target };
    p.current        = { g::kp_d, g::ki_d, g::kp_q, g::ki_q, m::Ld, m::Lq, m::psi_f, i::v_max };
    p.i_max          = g::i_max;
    return foc::FocController(p);
}

// ── Per-tick helpers ───────────────────────────────────────────────────────

// Pushes the true angle through the sim delay line, then compensates.
// encoderDelay is mutated each tick so it must be held in main().
inline foc::FocController::Measurements buildMeasurements(const sim::PMSM& motor,
                                                          sim::EncoderDelay& encoderDelay)
{
    foc::FocController::Measurements meas;
    meas.omega_m = static_cast<float>(motor.omega_m());
    meas.omega_e = static_cast<float>(cobalt::motor::pole_pairs) * meas.omega_m;

    const float theta_delayed = encoderDelay.push(static_cast<float>(motor.theta_e()));
    meas.theta_e = foc::compensateEncoderDelay(theta_delayed, meas.omega_e);

    motor.currents_alphabeta(meas.i_alpha, meas.i_beta);
    meas.vdc = cobalt::inverter::vdc;  // on hardware: read from bus voltage ADC each tick
    return meas;
}

inline void checkFaults(cobalt::Faults& faults, const sim::PMSM& motor, float vdc)
{
    const float speed_rpm = static_cast<float>(motor.omega_m()) * (60.0f / (2.0f * 3.14159265f));
    const float current_a = std::sqrt(motor.id() * motor.id() + motor.iq() * motor.iq());
    cobalt::updateFaults(faults, speed_rpm, vdc, current_a);
}

inline void stepPlant(sim::Inverter& simInv, sim::PMSM& motor,
                      float duty_a, float duty_b, float duty_c)
{
    double va, vb, vc;
    simInv.step(duty_a, duty_b, duty_c, va, vb, vc);

    float v_alpha, v_beta;
    foc::clarke(static_cast<float>(va), static_cast<float>(vb), v_alpha, v_beta);
    motor.step(sim_cfg::DT, v_alpha, v_beta);
}