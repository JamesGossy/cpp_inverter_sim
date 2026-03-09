// speed_controller.hpp
// Converts a speed error into a q-axis current request.

#pragma once
#include "pi_controller.hpp"

namespace foc {

class SpeedController {
public:

    struct Params {
        float kp;       // proportional gain
        float ki;       // integral gain
        float iq_max;   // output clamp — sets the peak current the speed loop can demand (A)
        Params() = default;
        Params(float kp, float ki, float iq_max) : kp(kp), ki(ki), iq_max(iq_max) {}
    };

    // PI output is clamped to [-iq_max, +iq_max].
    SpeedController(Params p) : pi(p.kp, p.ki, -p.iq_max, p.iq_max) {}

    // Returns iq_ref (A) for the given speed error (rad/s).
    float step(float targetSpeed, float actualSpeed, float dt) {
        return pi.step(targetSpeed - actualSpeed, dt);
    }

    void reset() { pi.reset(); }

private:
    PIController pi;
};

}