// speed_controller.hpp
// Outer speed loop: converts a speed error into a q-axis current request.

#pragma once
#include <algorithm>
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

    // Output is clamped to [0, iq_max] — q-axis current is always positive.
    SpeedController(Params p): iq_max(p.iq_max), pi(p.kp, p.ki, 0.0f, p.iq_max) {}

    // iq_limit lets field weakening reduce the current budget. Omit otherwise.
    float step(float targetSpeed, float actualSpeed, float dt, float iq_limit = -1.0f) {
        float limit = (iq_limit < 0.0f) ? iq_max : std::min(iq_limit, iq_max);
        return pi.step(targetSpeed - actualSpeed, dt, 0.0f, limit);
    }

    void reset() { pi.reset(); }

private:
    float iq_max = 0.0f;
    PIController pi;
};

}