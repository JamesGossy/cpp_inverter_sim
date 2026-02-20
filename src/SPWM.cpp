#include "SPWM.hpp"
#include <cmath>

namespace olinv {

Vec3 SPWM::duty_from_phase_voltage(const Vec3& v_phase_ref) const {
    const double half_vdc = 0.5 * vdc_;

    // Modulation index per phase, clamp to avoid overmodulation
    const double m_a = clamp(v_phase_ref.a / half_vdc, -0.999, 0.999);
    const double m_b = clamp(v_phase_ref.b / half_vdc, -0.999, 0.999);
    const double m_c = clamp(v_phase_ref.c / half_vdc, -0.999, 0.999);

    // For symmetric triangle carrier: duty = (m + 1)/2
    const double d_a = 0.5 * (m_a + 1.0);
    const double d_b = 0.5 * (m_b + 1.0);
    const double d_c = 0.5 * (m_c + 1.0);

    return {d_a, d_b, d_c};
}

double SPWM::triangle(double carrier_phase) {
    // Symmetric triangle in [-1, 1] for carrier_phase in [0,1)
    carrier_phase -= std::floor(carrier_phase);
    if (carrier_phase < 0.5) {
        return -1.0 + 4.0 * carrier_phase;
    }
    return 3.0 - 4.0 * carrier_phase;
}

} // namespace olinv