#pragma once
#include "Vec.hpp"

namespace olinv {

// Average-value 3-phase inverter with floating neutral.
// Input: duty (0..1) for each phase leg.
// Output: phase-to-neutral voltages (v_a, v_b, v_c) with v_a+v_b+v_c=0.
class AverageInverter {
public:
    explicit AverageInverter(double vdc) : vdc_(vdc) {}

    void set_vdc(double vdc) { vdc_ = vdc; }

    Vec3 phase_voltages_from_duty(const Vec3& duty_abc) const;

private:
    double vdc_{48.0};
};

} // namespace olinv