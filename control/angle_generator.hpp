// angle_generator.hpp
// Tracks an electrical angle that advances each time step.

#pragma once
#include "transforms.hpp"

namespace foc {

class AngleGenerator {
public:

    void reset(float startAngle = 0.0f) {
        angle = wrap_0_to_2pi(startAngle);
    }

    void step(float speed, float dt) {
        angle = wrap_0_to_2pi(angle + speed * dt);
    }

    float theta() const {
        return angle;
    }

private:
    float angle = 0.0f;
};

}