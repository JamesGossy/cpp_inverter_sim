#pragma once
#include <cmath>
#include <algorithm>

namespace olinv {

struct Vec2 {
    double x{0.0};
    double y{0.0};

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }
    Vec2& operator*=(double s) { x *= s; y *= s; return *this; }

    double norm() const { return std::sqrt(x*x + y*y); }
};

struct Vec3 {
    double a{0.0};
    double b{0.0};
    double c{0.0};

    Vec3() = default;
    Vec3(double a_, double b_, double c_) : a(a_), b(b_), c(c_) {}

    Vec3 operator+(const Vec3& o) const { return {a + o.a, b + o.b, c + o.c}; }
    Vec3 operator-(const Vec3& o) const { return {a - o.a, b - o.b, c - o.c}; }
    Vec3 operator*(double s) const { return {a * s, b * s, c * s}; }
};

inline double wrap_2pi(double theta) {
    constexpr double TWO_PI = 6.2831853071795864769;
    theta = std::fmod(theta, TWO_PI);
    if (theta < 0.0) theta += TWO_PI;
    return theta;
}

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
}

} // namespace olinv