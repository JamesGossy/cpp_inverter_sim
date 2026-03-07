// current_controller.hpp
// Dual-axis (d/q) current controller with optional back-EMF decoupling feedforward.
// Voltage output is limited as a vector (sqrt(vd^2 + vq^2) <= v_lim) rather than
// independently per axis, preserving the direction of the voltage request.
//
// Gains are calculated by tune_gains.py using pole-zero cancellation:
//   kp = L * omega_cc,   ki = Rs * omega_cc

#pragma once
#include <cmath>
#include "pi_controller.hpp"

namespace foc {

// Set to 1.0f for full SVPWM range, or 0.95f to keep a headroom margin.
static constexpr float kVoltageClampFactor = 1.0f;

class CurrentController {
public:
    struct Params {
        float kp_d, ki_d;
        float kp_q, ki_q;
        float Ld, Lq, psi_f;
        float v_max;
    };

    explicit CurrentController(const Params& p)
        : p_(p),
          v_lim_(p.v_max * kVoltageClampFactor),
          pi_d_(p.kp_d, p.ki_d, -v_lim_, v_lim_),
          pi_q_(p.kp_q, p.ki_q, -v_lim_, v_lim_) {}

    void step(float id_ref, float iq_ref,
              float id,     float iq,
              float omega_e, float dt,
              float& vd_out, float& vq_out)
    {
        // ── Decoupling feedforward ────────────────────────────────────────────
        const float ff_d = -omega_e * p_.Lq * iq;              // d-axis: cross-coupling
        const float ff_q =  omega_e * (p_.Ld * id + p_.psi_f); // q-axis: cross-coupling + back-EMF
        //const float ff_d = 0.0f;
        //const float ff_q = 0.0f;

        vd_out = pi_d_.step(id_ref - id, dt) + ff_d;
        vq_out = pi_q_.step(iq_ref - iq, dt) + ff_q;

        // ── Voltage vector limiting ───────────────────────────────────────────
        const float v_mag = std::sqrt(vd_out * vd_out + vq_out * vq_out);
        if (v_mag > v_lim_) {
            const float scale = v_lim_ / v_mag;
            vd_out *= scale;
            vq_out *= scale;
        }
    }

    void reset() { pi_d_.reset(); pi_q_.reset(); }

private:
    const Params p_;
    const float  v_lim_;
    PIController pi_d_, pi_q_;
};

} 