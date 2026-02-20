#include "PIController.hpp"

namespace olinv {

double PIController::update(double error, double dt) {
    const double p = kp_ * error;

    const double i_candidate = i_ + ki_ * error * dt;
    const double u_candidate = p + i_candidate;

    const double u_sat = clamp(u_candidate, lim_.u_min, lim_.u_max);
    const bool saturated = (u_sat != u_candidate);

    if (!saturated) {
        i_ = i_candidate;
        return u_candidate;
    }

    // Conditional integration (anti-windup):
    // if saturated high, only integrate when error would reduce output
    // if saturated low, only integrate when error would increase output
    if ((u_candidate > lim_.u_max && error < 0.0) ||
        (u_candidate < lim_.u_min && error > 0.0)) {
        i_ = i_candidate;
    }

    return u_sat;
}

} // namespace olinv