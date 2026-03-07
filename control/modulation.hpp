// modulation.hpp
// SVPWM modulator: takes phase voltage references and outputs duty cycles [0..1].
// Uses min-max zero-sequence injection to maximise the linear modulation range.
// Requires: <math.h>

#pragma once
#include <math.h>

namespace foc {

class SVPWMModulator {
public:
    SVPWMModulator(float v_dc) : v_dc_(v_dc) {}

    // Convert phase voltage references (V) to PWM duty cycles [0..1].
    void step(float va, float vb, float vc, float& duty_a, float& duty_b, float& duty_c) {

        // Zero-sequence injection: shift all three phases by the midpoint
        // of the max and min so the centred waveform fits inside the bus voltage.
        const float v_max    = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
        const float v_min    = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
        const float v_offset = -0.5f * (v_max + v_min);

        duty_a = clampf(0.5f + (va + v_offset) / v_dc_);
        duty_b = clampf(0.5f + (vb + v_offset) / v_dc_);
        duty_c = clampf(0.5f + (vc + v_offset) / v_dc_);
    }

    float vdc() const { return v_dc_; }

private:
    float v_dc_;

    static float clampf(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }
};

} 