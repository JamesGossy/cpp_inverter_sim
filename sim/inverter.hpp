// inverter.hpp
// Continuous (average) model of a 2-level 3-phase inverter.
// Converts duty cycles to phase-to-neutral voltages for simulation.

#pragma once

namespace sim {

class inverter {
public:
    inverter(double v_dc) : v_dc_(v_dc) {}

    void step(double duty_a, double duty_b, double duty_c, double& va,    double& vb,    double& vc) const {
        va = (2.0 * duty_a - 1.0) * (v_dc_ * 0.5);
        vb = (2.0 * duty_b - 1.0) * (v_dc_ * 0.5);
        vc = (2.0 * duty_c - 1.0) * (v_dc_ * 0.5);

        const double v_n = (va + vb + vc) / 3.0;  // Remove common-mode
        va -= v_n;  vb -= v_n;  vc -= v_n;
    }

private:
    double v_dc_;
};

}