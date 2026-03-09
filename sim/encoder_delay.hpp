// encoder_delay.hpp
// Simulates a fixed angle measurement latency using a FIFO queue.

#pragma once
#include <queue>

namespace sim {

class EncoderDelay {
public:

    // Pre-fills the queue with zeros to the required depth so push() is valid from tick 0.
    EncoderDelay(float delay_s, float dt)
    {
        int depth = static_cast<int>(delay_s / dt + 0.5f);  // number of ticks of latency
        for (int i = 0; i < depth; ++i)
            buf.push(0.0f);
    }

    // Feed in the true angle, get back the delayed angle.
    float push(float theta)
    {
        buf.push(theta);
        float delayed = buf.front();
        buf.pop();
        return delayed;
    }

private:
    std::queue<float> buf;  // FIFO of angle samples, length = delay in ticks
};

} 