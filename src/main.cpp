#include "Sim.hpp"

int main() {
    using namespace olinv;

    SimParams sp;
    sp.t_end = 0.25;
    sp.f_ctrl = 20000.0;     // 20 kHz
    sp.substeps = 10;        // 2 us sim step if 20kHz -> 50us period / 10 = 5us (note: 50us/10 = 5us)
    sp.Vdc = 48.0;
    sp.id_ref = 0.0;
    sp.iq_ref_step = 25.0;
    sp.iq_ref_ramp_time = 0.02;
    sp.csv_path = "../results/results.csv";

    PMSMParams mp;
    mp.R = 0.05;
    mp.Ld = 220e-6;
    mp.Lq = 220e-6;
    mp.psi_f = 0.015;
    mp.pole_pairs = 7;
    mp.J = 1.2e-4;
    mp.B = 1.0e-4;
    mp.T_load = 0.05;

    Sim sim(sp, mp);
    sim.run();
    return 0;
}