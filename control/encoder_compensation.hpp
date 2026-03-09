// encoder_compensation.hpp
// Compensates for the known angle measurement latency by extrapolating
// the delayed encoder reading forward using the current electrical speed.

#pragma once
#include "control/transforms.hpp"
#include "config/cobalt_params.hpp"

namespace foc {

inline float compensateEncoderDelay(float theta_delayed, float omega_e)
{
    return wrap_0_to_2pi(theta_delayed + omega_e * cobalt::encoder::delay_s);
}

} 