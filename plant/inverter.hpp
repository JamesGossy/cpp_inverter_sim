// inverter.hpp
// Converts PWM duty cycles (0 to 1) into balanced phase voltages for simulation.

#pragma once

namespace sim {

class Inverter {
public:

    Inverter(double busVoltage) : vdc(busVoltage) {}

    void step(double duty_a, double duty_b, double duty_c, double& va, double& vb, double& vc)
    {
        // Convert duty cycle to voltage centred on zero
        va = (2.0 * duty_a - 1.0) * (vdc * 0.5);
        vb = (2.0 * duty_b - 1.0) * (vdc * 0.5);
        vc = (2.0 * duty_c - 1.0) * (vdc * 0.5);

        // Remove common-mode so the phases sum to zero
        double avg = (va + vb + vc) / 3.0;
        va -= avg;
        vb -= avg;
        vc -= avg;
    }

private:
    double vdc;
};

}