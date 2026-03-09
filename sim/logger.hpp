// logger.hpp
// Writes simulation state to a CSV file at a configurable rate.

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
    // Creates the output directory if needed, opens the file, and writes the CSV header.
    // logEvery: write one row every N calls to log() to keep file size manageable.
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

    // Logs one row of motor state, skipping rows according to the logging rate.
    // va/vb/vc are the phase voltages applied by the inverter this tick.
    void log(const PMSM& motor, double t, double speedRef, float id_ref, float iq_ref, float duty_a, float duty_b, float duty_c, double va, double vb, double vc)
    {
        if (++count < every) return;
        count = 0;

        // Get stator currents in α/β; derive ia/ib/ic via inverse Clarke.
        double i_alpha, i_beta;
        motor.currents_alphabeta(i_alpha, i_beta);
        double ia =  i_alpha;
        double ib = -0.5 * i_alpha + 0.5 * std::sqrt(3.0) * i_beta;
        double ic = -0.5 * i_alpha - 0.5 * std::sqrt(3.0) * i_beta;

        const double rpm_scale = 60.0 / (2.0 * foc::PI);

        file << t
             << "," << speedRef * rpm_scale
             << "," << motor.omega_m() * rpm_scale
             << "," << motor.theta_e()
             << "," << va << "," << vb << "," << vc
             << "," << duty_a << "," << duty_b << "," << duty_c
             << "," << ia << "," << ib << "," << ic
             << "," << i_alpha << "," << i_beta
             << "," << id_ref << "," << motor.get_id()
             << "," << iq_ref << "," << motor.get_iq()
             << "," << motor.torque() << "\n";
    }

private:
    std::ofstream file;
    int every;      
    int count = 0;  
};

} 