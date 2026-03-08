// modulation.hpp
// Converts three target phase voltages into PWM duty cycles (0 to 1)
// using space vector PWM.

#pragma once
#include <math.h>

namespace foc {

class SVPWMModulator {
public:

    SVPWMModulator() = default;

    // Maximum phase voltage achievable with SVPWM (Vdc / √3).
    // Call this to get the voltage limit to pass into the current controller.
    static float voltage_limit(float vdc) {
        return vdc * 0.5773503f;  // vdc / √3
    }

    // vdc is passed per-call so it tracks a live bus voltage measurement.
    void step(float va, float vb, float vc, float vdc, float& duty_a, float& duty_b, float& duty_c)
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

private:

    float clamp(float x) {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }
};

}