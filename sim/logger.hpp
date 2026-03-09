// sim/logger.hpp
// Writes simulation state to a CSV file at a configurable decimation rate.

#pragma once
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "plant/pmsm.hpp"
#include "control/transforms.hpp"

namespace sim {

class Logger {
public:
    Logger(const std::string& path, int logEvery = 20) : every(logEvery) {
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());
        file.open(path);
        if (!file)
            std::cerr << "Logger: failed to open " << path << "\n";
        file << "t,rpm_ref,rpm,theta_e,"
                "va,vb,vc,duty_a,duty_b,duty_c,"
                "ia,ib,ic,ialpha,ibeta,"
                "id_ref,id_true,iq_ref,iq_true,Te\n"
             << std::fixed << std::setprecision(8);
    }

    void log(const PMSM& motor, double t,
             double speedRef,
             float id_ref, float iq_ref,
             float duty_a, float duty_b, float duty_c,
             double va, double vb, double vc)
    {
        if (++count < every) return;
        count = 0;

        double ia_d, ib_d;
        motor.currents_alphabeta(ia_d, ib_d);

        const double rpm_scale = 60.0 / (2.0 * foc::PI);

        file << t
             << "," << speedRef * rpm_scale
             << "," << motor.omega_m() * rpm_scale
             << "," << motor.theta_e()
             << "," << va << "," << vb << "," << vc
             << "," << duty_a << "," << duty_b << "," << duty_c
             << "," << ia_d
             << "," << (-0.5 * ia_d + 0.5 * std::sqrt(3.0) * ib_d)
             << "," << (-0.5 * ia_d - 0.5 * std::sqrt(3.0) * ib_d)
             << "," << ia_d << "," << ib_d
             << "," << id_ref << "," << motor.id()
             << "," << iq_ref << "," << motor.iq()
             << "," << motor.torque() << "\n";
    }

private:
    std::ofstream file;
    int every;
    int count = 0;
};

} // namespace sim