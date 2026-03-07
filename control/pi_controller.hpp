// pi_controller.hpp
// A PI controller: calculates a correction based on how far off we are (error)

#pragma once

namespace foc {

class PIController {
public:

    // Set up the controller with gains and output limits
    PIController(float kp, float ki, float outMin, float outMax) {
        this->kp = kp;
        this->ki = ki;
        this->outMin = outMin;
        this->outMax = outMax;
    }

    // Default constructor with no arguments
    PIController() {}

    // Run one step of the controller. Returns how much to correct by.
    float step(float error, float dt) {

        // Add to the running total (integral) over time
        integral += ki * error * dt;

        // Clamp the integral so it doesn't grow out of control (anti-windup)
        integral = clamp(integral, outMin, outMax);

        // Add the proportional part and clamp the final output too
        float output = kp * error + integral;
        return clamp(output, outMin, outMax);
    }

    // Reset the integral back to zero 
    void reset(float startValue = 0.0f) {
        integral = startValue;
    }

    // Get the current integral value
    float getIntegral() {
        return integral;
    }

    // Update the P and I gains
    void setGains(float newKp, float newKi) {
        kp = newKp;
        ki = newKi;
    }

    // Update the output limits
    void setLimits(float newMin, float newMax) {
        outMin = newMin;
        outMax = newMax;
    }

private:
    float kp       = 0.0f;    // proportional gain
    float ki       = 0.0f;    // integral gain
    float integral = 0.0f;    // running total of error over time
    float outMin   = -1e9f;   // minimum allowed output
    float outMax   =  1e9f;   // maximum allowed output

    // Keeps a value between lo and hi
    float clamp(float x, float lo, float hi) {
        if (x < lo) return lo;
        if (x > hi) return hi;
        return x;
    }
};

}