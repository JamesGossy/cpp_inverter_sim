#include "Logger.hpp"
#include <iomanip>

namespace olinv {

Logger::Logger(const std::string& csv_path) : ofs_(csv_path) {
    ofs_ << std::fixed << std::setprecision(9);
}

Logger::~Logger() {
    if (ofs_.is_open()) ofs_.close();
}

void Logger::write_header() {
    ofs_ << "t,"
         << "id_ref,iq_ref,"
         << "id,iq,"
         << "ia,ib,ic,"
         << "vd_cmd,vq_cmd,"
         << "va_ref,vb_ref,vc_ref,"
         << "da,db,dc,"
         << "va,vb,vc,"
         << "torque_e,"
         << "omega_m"
         << "\n";
}

void Logger::log(double t,
                 const Vec2& i_dq_ref,
                 const Vec2& i_dq,
                 const Vec3& i_abc,
                 const Vec2& v_dq_cmd,
                 const Vec3& v_abc_ref,
                 const Vec3& duty_abc,
                 const Vec3& v_abc_applied,
                 double torque_e,
                 double omega_m) {
    ofs_ << t << ","
         << i_dq_ref.x << "," << i_dq_ref.y << ","
         << i_dq.x << "," << i_dq.y << ","
         << i_abc.a << "," << i_abc.b << "," << i_abc.c << ","
         << v_dq_cmd.x << "," << v_dq_cmd.y << ","
         << v_abc_ref.a << "," << v_abc_ref.b << "," << v_abc_ref.c << ","
         << duty_abc.a << "," << duty_abc.b << "," << duty_abc.c << ","
         << v_abc_applied.a << "," << v_abc_applied.b << "," << v_abc_applied.c << ","
         << torque_e << ","
         << omega_m
         << "\n";
}

} // namespace olinv