// field_weakening_controller.hpp
// Outputs a negative d-axis current when the voltage approaches the inverter limit,
// allowing the motor to run faster than it otherwise could.

#pragma once
#include "pi_controller.hpp"

namespace foc {

class FieldWeakeningController {
public:

    struct Params {
        float kp;
        float ki;
        float id_min;
        float id_max;
        float voltage_target;
        Params() = default;
        Params(float kp, float ki, float id_min, float id_max, float voltage_target) : kp(kp), ki(ki), id_min(id_min), id_max(id_max), voltage_target(voltage_target) {}
    };

    FieldWeakeningController() {}

    FieldWeakeningController(Params p)
        : params(p), pi(p.kp, p.ki, p.id_min, p.id_max) {}

    // Returns the d-axis current reference.
    // voltageMagnitude is the applied dq voltage from the previous step.
    float step(float voltageMagnitude, float voltageLimit, float dt) {
        float target = params.voltage_target * voltageLimit;
        return pi.step(target - voltageMagnitude, dt);
    }

    void reset() {
        pi.reset(0.0f);
    }

private:
    Params params{};
    PIController pi;
};

}