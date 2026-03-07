// pi_controller.hpp
// Generic discrete PI controller with output clamping and integrator anti-windup.
// Anti-windup strategy: clamp the integrator state to [out_min, out_max] each step,
// so the integral cannot wind beyond what the output limiter will ever allow.

#pragma once

namespace foc {

class PIController {
public:
    PIController() = default;

    PIController(float kp, float ki, float out_min, float out_max)
        : kp_(kp), ki_(ki), out_min_(out_min), out_max_(out_max) {}

    // Advance by one time step. Returns the clamped controller output.
    float step(float error, float dt) {
        integral_ += ki_ * error * dt;
        integral_  = clamp(integral_, out_min_, out_max_);   // anti-windup
        return clamp(kp_ * error + integral_, out_min_, out_max_);
    }

    void  reset(float integral0 = 0.0f) { integral_ = integral0; }
    float integral() const { return integral_; }

    void set_gains (float kp, float ki)              { kp_ = kp; ki_ = ki; }
    void set_limits(float out_min, float out_max)    { out_min_ = out_min; out_max_ = out_max; }

private:
    float kp_      {0.0f};
    float ki_      {0.0f};
    float integral_{0.0f};
    float out_min_ {-1e9f};
    float out_max_ { 1e9f};

    static float clamp(float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
};

} 