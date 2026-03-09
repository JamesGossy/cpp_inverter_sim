// pi_controller.hpp
// PI controller with output clamping and integrator anti-windup.

#pragma once

namespace foc {

class PIController {
public:

    // outMin/outMax clamp both the integrator and the total output.
    PIController(float kp, float ki, float outMin, float outMax) : kp(kp), ki(ki), outMin(outMin), outMax(outMax) {}

    PIController() {}

    // Accumulates the integral, then returns kp*error + integral.
    // Both are clamped to [outMin, outMax] for anti-windup.
    float step(float error, float dt) {
        integral = clamp(integral + ki * error * dt, outMin, outMax);
        return clamp(kp * error + integral, outMin, outMax);
    }

    void  reset(float startValue = 0.0f) { integral = startValue; }
    float getIntegral() const { return integral; }
    void  setGains(float newKp, float newKi) { kp = newKp; ki = newKi; }
    void  setLimits(float newMin, float newMax) { outMin = newMin; outMax = newMax; }

private:
    float kp       = 0.0f;   // proportional gain
    float ki       = 0.0f;   // integral gain
    float integral = 0.0f;   // integrator state
    float outMin   = -1e9f;  // output and integrator lower clamp
    float outMax   =  1e9f;  // output and integrator upper clamp

    float clamp(float x, float lo, float hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }
};

}