// transforms.hpp
// Math helpers for FOC

#pragma once
#include <math.h>

namespace foc {

// Constants
const float PI = 3.14159265f;
const float TWO_PI = 2.0f * PI;
const float SQRT3 = 1.73205080f;

// Keeps an angle between 0 and 2*PI
inline float wrap_0_to_2pi(float angle) {
    angle = fmodf(angle, TWO_PI); // remove full rotations
    if (angle < 0) angle += TWO_PI; // make sure it's positive
    return angle;
}

// Convert 3-phase motor currents (a, b, c) into 2 values (alpha, beta)
inline void clarke(float a, float b, float& alpha, float& beta) {
    alpha = a;
    beta  = (a + 2.0f * b) / SQRT3;
}

// Reverse of clarke: convert alpha/beta back into 3-phase values
inline void inv_clarke(float alpha, float beta, float& a, float& b, float& c) {
    a =  alpha;
    b = -0.5f * alpha + 0.5f * SQRT3 * beta;
    c = -0.5f * alpha - 0.5f * SQRT3 * beta;
}

// Rotate alpha/beta by the rotor angle to get d/q values
inline void park(float alpha, float beta, float angle, float& d, float& q) {
    float c = cosf(angle);
    float s = sinf(angle);
    d =  c * alpha + s * beta;
    q = -s * alpha + c * beta;
}

// Reverse of park: convert d/q back into alpha/beta
inline void inv_park(float d, float q, float angle, float& alpha, float& beta) {
    float c = cosf(angle);
    float s = sinf(angle);
    alpha = c * d - s * q;
    beta  = s * d + c * q;
}

}