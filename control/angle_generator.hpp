// angle_generator.hpp
// Integrates electrical angle open-loop: theta += omega * dt.
// Requires: transforms.hpp

#pragma once
#include "transforms.hpp"

namespace foc {

class AngleGenerator {
public:
    void reset(float theta0_rad = 0.0f) {theta_ = wrap_0_to_2pi(theta0_rad);}

    void step(float omega_rad_s, float dt_s) {theta_ = wrap_0_to_2pi(theta_ + omega_rad_s * dt_s);}

    float theta() const { return theta_; }

private:
    float theta_{0.0f};
};

}