# CPP Inverter Sim

Closed-loop FOC simulation of a PMSM, written in C++ with a Python visualisation script. The control code is written to be shared directly with the MCU firmware, all parameters, transforms, and controller logic compile on both the sim and the target.

---

## Overview

The simulation runs three concurrently-scheduled loops at different rates:

| Loop | Rate | Responsibilities |
|---|---|---|
| Plant | 1 MHz | Inverter + PMSM integration |
| Inner | 20 kHz | Clarke/Park transforms, current controllers, SVPWM |
| Outer | 2 kHz | Speed controller, field weakening |

The plant integrates the motor voltage equations at 1 µs timesteps using phase voltages derived from the PWM duty cycles. The inner and outer loops fire on a stride basis against the plant tick counter.

---

## Control Structure

```
speed_ref ──► SpeedController ──► iq_ref ──►┐
                                             ├──► CurrentController ──► SVPWM ──► Motor
             FieldWeakening ──► id_ref ──────┘
```

**Field weakening** monitors |vdq| from the previous inner step and drives `id` negative as the voltage approaches the inverter limit, extending the achievable speed range beyond base speed.

**Encoder compensation** extrapolates the delayed angle measurement forward using the current electrical speed to reduce the effective latency seen by the Park transform.

---

## Motor Parameters

Defined in `config/cobalt_params.hpp` and shared between sim and firmware.

| Parameter | Value |
|---|---|
| Pole pairs | 5 |
| Winding resistance Rs | 0.138 Ω |
| d-axis inductance Ld | 520 µH |
| q-axis inductance Lq | 550 µH |
| PM flux linkage ψf | 0.051 Wb |
| Rotor inertia J | 2.0 × 10⁻⁴ kg·m² |
| DC bus voltage | 480 V |
| Peak current limit | 60 A |
| Encoder delay | 75 µs |

---

## Fault Protection

Faults latch on threshold breach and halt the inverter. Thresholds are defined in `cobalt_params.hpp`.

| Fault | Threshold |
|---|---|
| Overspeed | 20,000 RPM |
| Overvoltage | 560 V |
| Overcurrent | 75 A |
| Motor overtemp | 140 °C |
| Ambient overtemp | 70 °C |
| Module overtemp | 100 °C |

Temperature faults default to disabled in the sim until thermal modelling is added.

---

## File Structure

```
config/
  cobalt_params.hpp       — motor, inverter, gain, and fault parameters (shared with firmware)

control/
  foc_controller.hpp      — top-level FOC controller (outer + inner loop)
  current_controller.hpp  — d/q PI controllers with back-EMF feedforward
  speed_controller.hpp    — outer speed loop
  field_weakening_controller.hpp
  modulation.hpp          — SVPWM duty cycle generation
  transforms.hpp          — Clarke, Park, and angle utilities
  encoder_compensation.hpp
  pi_controller.hpp       — generic PI with clamping and anti-windup
  angle_generator.hpp

plant/
  pmsm.hpp / pmsm.cpp     — PMSM voltage equation model
  inverter.hpp            — duty cycle to phase voltage conversion

sim/
  encoder_delay.hpp       — ring buffer simulating angle measurement latency
  logger.hpp              — decimated CSV logger

system/
  faults.hpp              — latching fault flags and update logic

scripts/
  visualise_data.py       — plots results from sim_data.csv
  results/
    sim_data.csv          — simulation output (generated on run)

sim_main.cpp              — simulation entry point
sim_main.hpp              — configuration, factory functions, and per-tick helpers
```

---

## Visualisation

Requires `pandas` and `matplotlib`.

```bash
pip install pandas matplotlib
python scripts/visualise_data.py
```

Plots generated:
- Mechanical speed (ref vs actual)
- d-axis current (ref vs actual, side by side)
- q-axis current (ref vs actual, side by side)
- Torque
- Phase currents (ia, ib, ic)
- Phase voltages (va, vb, vc)
- Electrical angle
- PWM duty cycles
