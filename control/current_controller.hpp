// current_controller.hpp
// Controls the d and q axis currents using two PI controllers.
// Also adds a feedforward voltage to cancel out motor back-EMF.

#pragma once
#include <cmath>
#include "pi_controller.hpp"

namespace foc {

class CurrentController {
public:

    // Controller settings
    struct Params {
        float kp_d, ki_d;   // PI gains for the d-axis
        float kp_q, ki_q;   // PI gains for the q-axis
        float Ld, Lq;       // motor inductances 
        float psi_f;        // motor flux linkage
        float v_max;        // maximum voltage we're allowed to output
    };

    // Set up the controller with the given parameters
    CurrentController(Params p) {
        params = p;
        vLimit = p.v_max * 0.95;

        // Set up each PI controller with its gains and voltage limits
        pi_d = PIController(p.kp_d, p.ki_d, -vLimit, vLimit);
        pi_q = PIController(p.kp_q, p.ki_q, -vLimit, vLimit);
    }

    // Run one step: given target and actual d/q currents, output the voltages to apply
    void step(float id_ref, float iq_ref,
              float id,     float iq,
              float omega, float dt,
              float& vd_out, float& vq_out)
    {
        // Calculate feedforward voltages to cancel out the motor's back-EMF.
        // Without this, the PI controllers would have to fight against it alone.
        float ff_d = -omega * params.Lq * iq;
        float ff_q =  omega * (params.Ld * id + params.psi_f);

        // Run the PI controllers on the error, then add the feedforward
        vd_out = pi_d.step(id_ref - id, dt) + ff_d;
        vq_out = pi_q.step(iq_ref - iq, dt) + ff_q;

        // If the total voltage vector is too large, scale both axes down together.
        // Scales together so the direction stays the same.
        float magnitude = std::sqrt(vd_out * vd_out + vq_out * vq_out);
        if (magnitude > vLimit) {
            float scale = vLimit / magnitude;
            vd_out *= scale;
            vq_out *= scale;
        }
    }

    // Reset both PI controllers back to zero
    void reset() {
        pi_d.reset();
        pi_q.reset();
    }

private:
    Params params;      // copy of all the motor/controller settings
    float vLimit;       // maximum output voltage magnitude
    PIController pi_d;  // PI controller for the d-axis
    PIController pi_q;  // PI controller for the q-axis
};

}