// foc_controller.hpp
// Consolidated FOC controller.
//
// Owns all control state and exposes two independently-steppable loops that
// mirror the real scheduling structure of the drive:
//
//   stepOuter()  –  field-weakening + speed controller (slow loop, e.g. 2 kHz)
//                   Reads voltage_mag from the previous inner step to drive FW.
//                   Writes id_ref, iq_ref consumed by the next inner step.
//
//   stepInner()  –  Park transform → current controllers → inv-Park →
//                   inv-Clarke → SVPWM  (fast loop, e.g. 20 kHz)
//                   Writes duty cycles and voltage_mag.
//
// The caller is responsible for scheduling (which tick each loop fires on)
// and for populating Measurements from sensors / the simulation plant.

#pragma once

#include <algorithm>
#include <cmath>

#include "current_controller.hpp"
#include "field_weakening_controller.hpp"
#include "modulation.hpp"
#include "speed_controller.hpp"
#include "transforms.hpp"

namespace foc {

class FocController {
public:

    // ── Parameter bundle ──────────────────────────────────────────────────
    struct Params {
        SpeedController::Params          speed;
        FieldWeakeningController::Params fieldWeakening;
        CurrentController::Params        current;
        float i_max = 0.0f;  // peak current envelope (A) shared between d and q
    };

    // ── Inputs ────────────────────────────────────────────────────────────
    // Populate from sensors (or simulation) before calling either step.
    struct Measurements {
        float theta_e = 0.0f;  // electrical angle (rad)
        float omega_m = 0.0f;  // mechanical speed  (rad/s)
        float omega_e = 0.0f;  // electrical speed  (rad/s)
        float i_alpha = 0.0f;  // stator current, stationary α frame (A)
        float i_beta  = 0.0f;  // stator current, stationary β frame (A)
        float vdc     = 0.0f;  // DC bus voltage from sensor (V)
    };

    struct References {
        float speed_ref = 0.0f;  // speed setpoint (rad/s)
    };

    // ── Outputs ───────────────────────────────────────────────────────────
    struct OuterLoopOutput {
        float voltage_limit = 0.0f;  // Vdc/√3 computed this step (V)
        float id_ref        = 0.0f;  // d-axis current reference from FW (A)
        float iq_ref        = 0.0f;  // q-axis current reference from speed ctrl (A)
    };

    struct InnerLoopOutput {
        float voltage_limit = 0.0f;  // Vdc/√3 computed this step (V)
        float id            = 0.0f;  // measured d-axis current (A)
        float iq            = 0.0f;  // measured q-axis current (A)
        float vd            = 0.0f;  // d-axis voltage command (V)
        float vq            = 0.0f;  // q-axis voltage command (V)
        float voltage_mag   = 0.0f;  // |vdq| — fed back into next outer step (V)
        float v_alpha       = 0.0f;
        float v_beta        = 0.0f;
        float va            = 0.0f;
        float vb            = 0.0f;
        float vc            = 0.0f;
        float duty_a        = 0.5f;
        float duty_b        = 0.5f;
        float duty_c        = 0.5f;
    };

    struct Output {
        OuterLoopOutput outer;
        InnerLoopOutput inner;
    };

    // ── Lifecycle ─────────────────────────────────────────────────────────
    FocController() = default;

    explicit FocController(Params p)
        : params(p),
          speedCtrl(p.speed),
          fwCtrl(p.fieldWeakening),
          currentCtrl(p.current) {}

    // ── Outer loop step (field-weakening + speed controller) ──────────────
    // Call at the slow rate (e.g. every 500 µs / 2 kHz).
    // Returns the updated outer-loop output (also held in state()).
    const OuterLoopOutput& stepOuter(const Measurements& meas, const References& refs, float dt)
    {
        output.outer.voltage_limit = SVPWMModulator::voltage_limit(meas.vdc);

        // Field weakening: drive id negative when voltage approaches the limit.
        // Uses voltage_mag from the previous inner step as the feedback signal.
        output.outer.id_ref = fwCtrl.step(output.inner.voltage_mag, output.outer.voltage_limit, dt);

        // Speed controller: produces iq_ref within its fixed current limit.
        output.outer.iq_ref = speedCtrl.step(refs.speed_ref, meas.omega_m, dt);

        return output.outer;
    }

    // ── Inner loop step (transforms + current control + modulation) ────────
    // Call at the fast rate (e.g. every 50 µs / 20 kHz).
    // Reads id_ref / iq_ref written by the most recent stepOuter().
    // Returns the updated inner-loop output (also held in state()).
    const InnerLoopOutput& stepInner(const Measurements& meas, float dt)
    {
        output.inner.voltage_limit = SVPWMModulator::voltage_limit(meas.vdc);

        // Park: stationary α/β → rotating d/q
        park(meas.i_alpha, meas.i_beta, meas.theta_e, output.inner.id, output.inner.iq);

        // Current controllers (PI + back-EMF feedforward, d-axis favoured)
        currentCtrl.step(output.outer.id_ref, output.outer.iq_ref, output.inner.id, output.inner.iq, meas.omega_e, output.inner.voltage_limit, dt, output.inner.vd, output.inner.vq);

        // |vdq| is fed back into the FW controller on the next outer step
        output.inner.voltage_mag = std::sqrt(output.inner.vd * output.inner.vd + output.inner.vq * output.inner.vq);

        // Inverse Park: d/q → α/β
        inv_park(output.inner.vd, output.inner.vq, meas.theta_e, output.inner.v_alpha, output.inner.v_beta);

        // Inverse Clarke: α/β → three-phase
        inv_clarke(output.inner.v_alpha, output.inner.v_beta, output.inner.va, output.inner.vb, output.inner.vc);

        // SVPWM modulation — vdc tracked live so duty cycles scale correctly
        mod.step(output.inner.va, output.inner.vb, output.inner.vc, meas.vdc, output.inner.duty_a, output.inner.duty_b, output.inner.duty_c);

        return output.inner;
    }

    // Full output snapshot (read-only)
    const Output& state() const { return output; }

    void reset() {
        speedCtrl.reset();
        fwCtrl.reset();
        currentCtrl.reset();
        output = {};
    }

private:
    Params                   params{};
    SpeedController          speedCtrl;
    FieldWeakeningController fwCtrl;
    CurrentController        currentCtrl;
    SVPWMModulator           mod;
    Output                   output{};
};

} // namespace foc