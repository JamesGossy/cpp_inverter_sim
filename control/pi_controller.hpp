// pi_controller.hpp
// A basic PI controller with output clamping and anti-windup.

#pragma once

namespace foc {

class PIController {
public:

    PIController(float kp, float ki, float outMin, float outMax) : kp(kp), ki(ki), outMin(outMin), outMax(outMax) {}

    PIController() {}

    // Run one control step and return the output.
    // dynMin/dynMax optionally tighten the output clamp for this step
    float step(float error, float dt, float dynMin = -1.0f, float dynMax = -1.0f) {
        integral = clamp(integral + ki * error * dt, outMin, outMax);

        float lo = outMin;
        float hi = outMax;
        if (dynMax >= 0.0f) {
            lo = (dynMin >= 0.0f) ? dynMin : -dynMax;
            hi = dynMax;
        }

        integral = clamp(integral, lo, hi);
        return clamp(kp * error + integral, lo, hi);
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