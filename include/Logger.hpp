#pragma once
#include <fstream>
#include <string>
#include "Vec.hpp"

namespace olinv {

class Logger {
public:
    explicit Logger(const std::string& csv_path);
    ~Logger();

    void write_header();

    void log(double t,
             const Vec2& i_dq_ref,
             const Vec2& i_dq,
             const Vec3& i_abc,
             const Vec2& v_dq_cmd,
             const Vec3& v_abc_ref,
             const Vec3& duty_abc,
             const Vec3& v_abc_applied,
             double torque_e,
             double omega_m);

private:
    std::ofstream ofs_;
};

} // namespace olinv