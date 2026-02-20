#pragma once
#include <string>
#include "Vec.hpp"
#include "PMSMModel.hpp"
#include "CurrentController.hpp"
#include "AverageInverter.hpp"
#include "SPWM.hpp"
#include "Logger.hpp"
#include "Transforms.hpp"

namespace olinv {

struct SimParams {
    double t_end{0.25};        // seconds
    double f_ctrl{20000.0};    // 20 kHz control + PWM update
    int substeps{10};          // sim substeps per control tick
    double Vdc{48.0};

    // Open-loop reference shaping (iq command)
    double id_ref{0.0};
    double iq_ref_step{20.0};
    double iq_ref_ramp_time{0.02};

    std::string csv_path{"results.csv"};
};

class Sim {
public:
    Sim(const SimParams& sp, const PMSMParams& mp);
    void run();

private:
    SimParams sp_;
    PMSMModel motor_;
    CurrentController cc_;
    AverageInverter inv_;
    SPWM spwm_;
    Logger logger_;

    Vec2 current_ref(double t) const;
};

} // namespace olinv