// sim_main.cpp
// Closed-loop FOC simulation of a PMSM.
//
//  Outer loop  –   2 kHz  (every 500 ticks) : field-weakening + speed controller
//  Inner loop  –  20 kHz  (every  50 ticks) : transforms, current controllers, SVPWM, faults
//  Plant       –   1 MHz  (every tick)      : inverter + PMSM integration

#include <iostream>
#include "sim_main.hpp"

int main()
{
    // ── Initialisation ────────────────────────────────────────────────────
    sim::PMSM motor = makePMSM();
    sim::Inverter simInv(cobalt::inverter::vdc);
    foc::FocController foc = makeFocController();
    sim::Logger logger("scripts/results/sim_data.csv", sim_cfg::LOG_EVERY);
    cobalt::Faults faults = {};
    sim::EncoderDelay encoderDelay(cobalt::encoder::delay_s, static_cast<float>(sim_cfg::DT));

    const double   TARGET_SPEED = sim_cfg::TARGET_RPM * 2.0 * foc::PI / 60.0;
    const uint64_t TOTAL_TICKS  = static_cast<uint64_t>(sim_cfg::T_END / sim_cfg::DT) + 1;

    // ── Main simulation loop (1 MHz plant tick) ───────────────────────────
    for (uint64_t n = 0; n < TOTAL_TICKS; ++n)
    {
        const double t = static_cast<double>(n) * sim_cfg::DT;

        // Hold at zero for 100 ms to let the controller initialise before ramping.
        const double speedRef = (t < 0.1) ? 0.0 : TARGET_SPEED;

        const auto meas = buildMeasurements(motor, encoderDelay);
        const foc::FocController::References refs = { static_cast<float>(speedRef) };

        // Outer loop: field-weakening + speed controller (2 kHz).
        if (n % sim_cfg::OUTER_STRIDE == 0)
            foc.stepOuter(meas, refs, sim_cfg::OUTER_DT);

        // Inner loop: current controllers + SVPWM + faults (20 kHz).
        if (n % sim_cfg::INNER_STRIDE == 0)
        {
            foc.stepInner(meas, sim_cfg::INNER_DT);

            checkFaults(faults, motor, meas.vdc);
            if (faults.any()) {
                std::cout << "Fault at t=" << t << "s, halting.\n";
                break;
            }
        }

        // Advance the plant using the most recent duty cycles.
        const auto& inner = foc.state().inner;
        stepPlant(simInv, motor, inner.duty_a, inner.duty_b, inner.duty_c);

        // Log at the specified rate defined by LOG_EVERY.
        const auto& outer = foc.state().outer;
        logger.log(motor, t, speedRef, outer.id_ref, outer.iq_ref, inner.duty_a, inner.duty_b, inner.duty_c, inner.va, inner.vb, inner.vc);
    }

    std::cout << "Done. Wrote results/sim_data.csv\n";
    return 0;
}