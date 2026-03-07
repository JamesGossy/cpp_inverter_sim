// speed_controller.hpp
// Outer speed loop: takes a target speed and outputs how much q-axis current to request

#pragma once
#include "pi_controller.hpp"

namespace foc {

class SpeedController {
public:

    // All the settings needed to set up the speed controller
    struct Params {
        float kp;      // proportional gain
        float ki;      // integral gain
        float iq_max;  // maximum current we're allowed to request
    };

    // Set up the controller with the given parameters
    SpeedController(Params p) {
        pi = PIController(p.kp, p.ki, -p.iq_max, p.iq_max);
    }

    // Run one step: given target and actual speed, returns the q-axis current to request
    float step(float targetSpeed, float actualSpeed, float dt) {
        float error = targetSpeed - actualSpeed;
        return pi.step(error, dt);
    }

    // Reset the PI controller back to zero
    void reset() {
        pi.reset();
    }

private:
    PIController pi;
};

}