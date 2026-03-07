// modulation.hpp
// Converts target phase voltages into PWM duty cycles (0 to 1)

#pragma once
#include <math.h>

namespace foc {

class SVPWMModulator {
public:

    // Set the bus voltage 
    SVPWMModulator(float busVoltage) {
        vdc = busVoltage;
    }

    // Takes the 3 target voltages and calculates a duty cycle for each phase
    void step(float va, float vb, float vc, float& duty_a, float& duty_b, float& duty_c) {

        // Find the highest and lowest of the three voltages
        float v_max = va;
        if (vb > v_max) v_max = vb;
        if (vc > v_max) v_max = vc;

        float v_min = va;
        if (vb < v_min) v_min = vb;
        if (vc < v_min) v_min = vc;

        // Shift all three voltages so they sit in the middle of the bus voltage.
        // This lets us get more range out of the same supply voltage.
        float offset = -0.5f * (v_max + v_min);

        // Convert each voltage to a duty cycle between 0 and 1
        duty_a = clamp(0.5f + (va + offset) / vdc);
        duty_b = clamp(0.5f + (vb + offset) / vdc);
        duty_c = clamp(0.5f + (vc + offset) / vdc);
    }

    // Get the bus voltage
    float getVdc() {
        return vdc;
    }

private:
    float vdc; // bus voltage 

    // Clamps a value to stay between 0 and 1
    float clamp(float x) {
        if (x < 0.0f) return 0.0f;
        if (x > 1.0f) return 1.0f;
        return x;
    }
};

}