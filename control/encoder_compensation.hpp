// control/encoder_compensation.hpp
// Compensates for the known angle measurement latency by extrapolating
// the delayed encoder reading forward using the current electrical speed.
//
//   theta_comp = wrap(theta_delayed + omega_e * delay_s)
//
// delay_s is the total latency from physical angle to the value available
// in the ISR: encoder read + any filtering + DMA/SPI transfer time.
// It is set in cobalt_params.hpp and shared with the sim.

#pragma once
#include "control/transforms.hpp"
#include "config/cobalt_params.hpp"

namespace foc {

inline float compensateEncoderDelay(float theta_delayed, float omega_e)
{
    return wrap_0_to_2pi(theta_delayed + omega_e * cobalt::encoder::delay_s);
}

} // namespace foc