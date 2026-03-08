// current_controller.hpp
// PI controllers for the d and q axis currents, with back-EMF feedforward.
// The d-axis is clamped first within the full voltage limit; the q-axis
// gets whatever headroom is left.

#pragma once
#include <algorithm>
#include <cmath>
#include "pi_controller.hpp"

namespace foc {

class CurrentController {
public:

    struct Params {
        float kp_d, ki_d;
        float kp_q, ki_q;
        float Ld, Lq;
        float psi_f;
        float v_max;
        Params() = default;
        Params(float kp_d, float ki_d, float kp_q, float ki_q, float Ld, float Lq, float psi_f, float v_max) : kp_d(kp_d), ki_d(ki_d), kp_q(kp_q), ki_q(ki_q), Ld(Ld), Lq(Lq), psi_f(psi_f), v_max(v_max) {}
    };

    CurrentController(Params p) : params(p) {
        vLimit = p.v_max;
        pi_d = PIController(p.kp_d, p.ki_d, -vLimit, vLimit);
        pi_q = PIController(p.kp_q, p.ki_q, -vLimit, vLimit);
    }

    void step(float id_ref, float iq_ref,
              float id, float iq,
              float omega, float dt,
              float& vd_out, float& vq_out)
    {
        // Feedforward terms to cancel the motor's back-EMF
        float ff_d = -omega * params.Lq * iq;
        float ff_q = omega * (params.Ld * id + params.psi_f);

        // D-axis uses the full voltage budget; q-axis uses what remains
        vd_out = std::clamp(pi_d.step(id_ref - id, dt) + ff_d, -vLimit, vLimit);

        float vq_budget = std::sqrt(std::max(0.0f, vLimit * vLimit - vd_out * vd_out));
        vq_out = std::clamp(pi_q.step(iq_ref - iq, dt, vq_budget) + ff_q, -vq_budget, vq_budget);
    }

    void reset() {
        pi_d.reset();
        pi_q.reset();
    }

private:
    Params params;
    float vLimit;
    PIController pi_d;
    PIController pi_q;
};

}