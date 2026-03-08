// speed_controller.hpp
// Outer speed loop: converts a speed error into a q-axis current request.

#pragma once
#include "pi_controller.hpp"

namespace foc {

class SpeedController {
public:

    struct Params {
        float kp;
        float ki;
        float iq_max;
        Params() = default;
        Params(float kp, float ki, float iq_max) : kp(kp), ki(ki), iq_max(iq_max) {}
    };

    SpeedController(Params p) : pi(p.kp, p.ki, -p.iq_max, p.iq_max) {}

    float step(float targetSpeed, float actualSpeed, float dt) {
        return pi.step(targetSpeed - actualSpeed, dt);
    }

    void reset() { pi.reset(); }

private:
    PIController pi;
};

}