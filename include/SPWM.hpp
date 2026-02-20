#pragma once
#include "vec.hpp"

namespace olinv {

// Sine PWM utilities.
class SPWM {
public:
    explicit SPWM(double vdc) : vdc_(vdc) {}
    void set_vdc(double vdc) { vdc_ = vdc; }

    // Convert phase voltage refs (phase-to-neutral) to duties (0..1).
    // This is equivalent to "compare sine with triangle" for average PWM.
    Vec3 duty_from_phase_voltage(const Vec3& v_phase_ref) const;

    // Triangle carrier in [-1, 1], carrier_phase in [0,1).
    static double triangle(double carrier_phase);

private:
    double vdc_{48.0};
};

} // namespace olinv