// inverter.hpp
// Converts PWM duty cycles (0 to 1) into phase voltages for simulation

#pragma once

namespace sim {

class Inverter {
public:

    // Set the bus voltage 
    Inverter(double busVoltage) {
        vdc = busVoltage;
    }

    // Takes 3 duty cycles and outputs the 3 phase voltages
    void step(double duty_a, double duty_b, double duty_c, double& va, double& vb, double& vc) {

        // Convert each duty cycle to a voltage (-vdc/2 to +vdc/2)
        va = (2.0 * duty_a - 1.0) * (vdc * 0.5);
        vb = (2.0 * duty_b - 1.0) * (vdc * 0.5);
        vc = (2.0 * duty_c - 1.0) * (vdc * 0.5);

        // Subtract the average so the three phases are balanced around zero
        double average = (va + vb + vc) / 3.0;
        va -= average;
        vb -= average;
        vc -= average;
    }

private:
    double vdc; // bus voltage
};

}