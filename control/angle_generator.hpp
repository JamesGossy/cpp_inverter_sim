// angle_generator.hpp
// Keeps track of an electrical angle that increases over time

#pragma once
#include "transforms.hpp"

namespace foc {

class AngleGenerator {
public:

    // Set the starting angle (defaults to 0)
    void reset(float startAngle = 0.0f) {
        angle = wrap_0_to_2pi(startAngle);
    }

    // Move the angle forward by one time step
    void step(float speed, float deltaTime) {
        angle = wrap_0_to_2pi(angle + speed * deltaTime);
    }

    // Get the current angle
    float theta() const {
        return angle;
    }

private:
    float angle = 0.0f; // current angle in radians
};

}