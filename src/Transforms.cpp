#include "Transforms.hpp"
#include <cmath>

namespace olinv {

Vec2 clarke(const Vec3& abc) {
    // alpha = i_a
    // beta  = (i_a + 2 i_b) / sqrt(3)
    constexpr double INV_SQRT3 = 0.5773502691896257645;
    const double alpha = abc.a;
    const double beta  = (abc.a + 2.0 * abc.b) * INV_SQRT3;
    return {alpha, beta};
}

Vec3 inv_clarke(const Vec2& ab) {
    // i_a = alpha
    // i_b = -0.5 alpha + sqrt(3)/2 beta
    // i_c = -0.5 alpha - sqrt(3)/2 beta
    constexpr double HALF = 0.5;
    constexpr double SQRT3_OVER_2 = 0.8660254037844386468;
    const double a = ab.x;
    const double b = -HALF * ab.x + SQRT3_OVER_2 * ab.y;
    const double c = -HALF * ab.x - SQRT3_OVER_2 * ab.y;
    return {a, b, c};
}

Vec2 park(const Vec2& ab, double theta_e) {
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    // [d]   [ c  s][alpha]
    // [q] = [-s  c][beta ]
    const double d =  c * ab.x + s * ab.y;
    const double q = -s * ab.x + c * ab.y;
    return {d, q};
}

Vec2 inv_park(const Vec2& dq, double theta_e) {
    const double c = std::cos(theta_e);
    const double s = std::sin(theta_e);
    // [alpha]   [ c -s][d]
    // [beta ] = [ s  c][q]
    const double alpha = c * dq.x - s * dq.y;
    const double beta  = s * dq.x + c * dq.y;
    return {alpha, beta};
}

} // namespace olinv