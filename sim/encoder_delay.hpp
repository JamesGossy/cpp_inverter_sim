// sim/encoder_delay.hpp
// Simulates a fixed angle measurement latency using a ring buffer.
// This is sim-only — on hardware the delay is physical and needs no modelling.
//
// Sized at construction from delay_s and the plant timestep dt.
// Call push() every plant tick: feed in the true angle, get back the delayed one.

#pragma once
#include <array>
#include <cstdint>

namespace sim {

class EncoderDelay {
public:

    // delay_s: total latency to simulate (e.g. cobalt::encoder::delay_s)
    // dt:      plant timestep (e.g. sim_cfg::DT)
    EncoderDelay(float delay_s, float dt)
        : depth(static_cast<int>(delay_s / dt + 0.5f))
    {}

    // Feed in the true angle, get back the delayed angle.
    float push(float theta)
    {
        buf[head] = theta;
        head = (head + 1) % depth;
        return buf[head];  // oldest entry = delayed output
    }

private:
    static constexpr int MAX_DEPTH = 512;
    std::array<float, MAX_DEPTH> buf = {};
    int head  = 0;
    int depth = 1;
};

} // namespace sim