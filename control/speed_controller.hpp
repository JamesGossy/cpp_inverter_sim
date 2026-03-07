// speed_controller.hpp
// Outer mechanical speed loop. Outputs iq_ref; id_ref is always zero (MTPA).
//
// Gains are calculated by tune_gains.py using pole-zero cancellation:
//   kp = J * omega_sc / Kt,   ki = B * omega_sc / Kt

#pragma once
#include "pi_controller.hpp"

namespace foc {

class SpeedController {
public:
    struct Params {
        float kp, ki, iq_max;
    };

    explicit SpeedController(const Params& p)
        : pi_(p.kp, p.ki, -p.iq_max, p.iq_max) {}

    float step(float omega_ref, float omega_m, float dt) {
        return pi_.step(omega_ref - omega_m, dt);
    }

    void reset() { pi_.reset(); }

private:
    PIController pi_;
};

} 