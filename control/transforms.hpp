// transforms.hpp
// Clarke (abc->αβ), Park (αβ->dq), and their inverses.
// Requires: <math.h>  (fmodf, cosf, sinf — must resolve on your target platform)

#pragma once
#include <math.h>

namespace foc {

constexpr float kPi       = 3.14159265f;
constexpr float kSqrt3    = 1.73205080f;
constexpr float kTwoPi    = 2.0f * kPi;
constexpr float kInvSqrt3 = 1.0f / kSqrt3;  // 1/√3 — used for SVPWM voltage limit

inline float wrap_0_to_2pi(float theta) {
    theta = fmodf(theta, kTwoPi);
    return (theta < 0.0f) ? theta + kTwoPi : theta;
}

// abc -> alpha/beta
inline void clarke(float a, float b, float& alpha, float& beta) {
    alpha = a;
    beta  = (a + 2.0f * b) / kSqrt3;
}

// alpha/beta -> abc
inline void inv_clarke(float alpha, float beta, float& a, float& b, float& c) {
    a =  alpha;
    b = -0.5f * alpha + 0.5f * kSqrt3 * beta;
    c = -0.5f * alpha - 0.5f * kSqrt3 * beta;
}

// alpha/beta -> d/q
inline void park(float alpha, float beta, float theta, float& d, float& q) {
    const float co = cosf(theta), si = sinf(theta);
    d =  co * alpha + si * beta;
    q = -si * alpha + co * beta;
}

// d/q -> alpha/beta
inline void inv_park(float d, float q, float theta, float& alpha, float& beta) {
    const float co = cosf(theta), si = sinf(theta);
    alpha = co * d - si * q;
    beta  = si * d + co * q;
}

}