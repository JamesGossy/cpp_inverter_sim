// pi_controller.hpp
// A basic PI controller with output clamping and anti-windup.

#pragma once

namespace foc {

class PIController {
public:

    PIController(float kp, float ki, float outMin, float outMax) : kp(kp), ki(ki), outMin(outMin), outMax(outMax) {}

    PIController() {}

    // Run one control step and return the output.
    // The integrator is clamped to [outMin, outMax] for anti-windup.
    float step(float error, float dt) {
        integral = clamp(integral + ki * error * dt, outMin, outMax);
        return clamp(kp * error + integral, outMin, outMax);
    }

    void  reset(float startValue = 0.0f) { integral = startValue; }
    float getIntegral() const { return integral; }
    void  setGains(float newKp, float newKi) { kp = newKp; ki = newKi; }
    void  setLimits(float newMin, float newMax) { outMin = newMin; outMax = newMax; }

private:
    float kp = 0.0f;
    float ki = 0.0f;
    float integral = 0.0f;
    float outMin = -1e9f;
    float outMax =  1e9f;

    float clamp(float x, float lo, float hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }
};

}