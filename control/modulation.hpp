// modulation.hpp
// Converts three target phase voltages into PWM duty cycles (0 to 1)
// using space vector PWM.

#pragma once
#include <math.h>

namespace foc {

class SVPWMModulator {
public:

    SVPWMModulator(float busVoltage) : vdc(busVoltage) {}

    void step(float va, float vb, float vc,
              float& duty_a, float& duty_b, float& duty_c)
    {
        // Find the mid-point of the three voltages and shift them all by it.
        // This centres the waveforms within the bus voltage range (SVPWM offset).
        float v_max = va > vb ? va : vb;
        if (vc > v_max) v_max = vc;

        float v_min = va < vb ? va : vb;
        if (vc < v_min) v_min = vc;

        float offset = -0.5f * (v_max + v_min);

        duty_a = clamp(0.5f + (va + offset) / vdc);
        duty_b = clamp(0.5f + (vb + offset) / vdc);
        duty_c = clamp(0.5f + (vc + offset) / vdc);
    }

    float getVdc() const { return vdc; }

private:
    float vdc;

    float clamp(float x) {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }
};

}