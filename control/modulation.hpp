// modulation.hpp
// Converts three target phase voltages into PWM duty cycles (0 to 1) using space vector PWM.

#pragma once
#include <math.h>

namespace foc {

class SVPWMModulator {
public:

    SVPWMModulator() = default;

    // Maximum phase voltage achievable with SVPWM (Vdc / √3).
    static float voltage_limit(float vdc) {
        return vdc * 0.5773503f;  // vdc / √3
    }

    // vdc is passed per-call so it tracks a live bus voltage measurement.
    void step(float va, float vb, float vc, float vdc, float& duty_a, float& duty_b, float& duty_c)
    {
        float v_max = va > vb ? va : vb;
        if (vc > v_max) v_max = vc;

        float v_min = va < vb ? va : vb;
        if (vc < v_min) v_min = vc;

        // Zero-sequence offset: shifts all three phases equally so the midpoint
        // of the min/max pair lands at zero, maximising linear modulation range.
        float offset = -0.5f * (v_max + v_min);

        // Normalise to [0, 1] around the 0.5 midpoint of the PWM carrier.
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