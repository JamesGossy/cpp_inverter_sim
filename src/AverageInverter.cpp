#include "AverageInverter.hpp"

namespace olinv {

Vec3 AverageInverter::phase_voltages_from_duty(const Vec3& duty_abc) const {
    // Leg average voltage w.r.t DC midpoint:
    // v_a0 = (2d_a - 1) * Vdc/2
    const double v_a0 = (2.0 * duty_abc.a - 1.0) * (vdc_ * 0.5);
    const double v_b0 = (2.0 * duty_abc.b - 1.0) * (vdc_ * 0.5);
    const double v_c0 = (2.0 * duty_abc.c - 1.0) * (vdc_ * 0.5);

    // Floating neutral: subtract common-mode so sum is zero
    const double v_cm = (v_a0 + v_b0 + v_c0) / 3.0;
    return {v_a0 - v_cm, v_b0 - v_cm, v_c0 - v_cm};
}

} // namespace olinv