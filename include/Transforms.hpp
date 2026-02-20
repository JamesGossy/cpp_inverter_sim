#pragma once
#include "Vec.hpp"

namespace olinv {

// Clarke: abc -> alpha-beta (balanced: i_a+i_b+i_c = 0)
Vec2 clarke(const Vec3& abc);

// Inverse Clarke: alpha-beta -> abc
Vec3 inv_clarke(const Vec2& ab);

// Park: alpha-beta -> dq (rotating by theta_e)
Vec2 park(const Vec2& ab, double theta_e);

// Inverse Park: dq -> alpha-beta
Vec2 inv_park(const Vec2& dq, double theta_e);

} // namespace olinv