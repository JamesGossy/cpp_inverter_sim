// config/cobalt_params.hpp
// Hardware and tuning parameters for the Cobalt motor drive.
// Shared between the sim and MCU firmware.

#pragma once

namespace cobalt {

namespace motor {
    constexpr int   pole_pairs = 5;
    constexpr float Rs         = 0.138f;   // winding resistance (Ω)
    constexpr float Ld         = 520e-6f;  // d-axis inductance (H)
    constexpr float Lq         = 550e-6f;  // q-axis inductance (H)
    constexpr float psi_f      = 0.051f;   // PM flux linkage (Wb)
    constexpr float J          = 2.0e-4f;  // rotor inertia (kg·m²)
    constexpr float B          = 2.0e-4f;  // viscous friction coefficient
    constexpr float T_load     = 0.1f;     // constant load torque (N·m)
}

namespace inverter {
    constexpr float vdc   = 480.0f;
    constexpr float v_max = vdc / 1.7320508f;  // Vdc / √3 (V)
}

namespace gains {
    // Current controller (inner loop, 20 kHz)
    constexpr float kp_d  = 8.0f;
    constexpr float ki_d  = 1500.0f;
    constexpr float kp_q  = 8.0f;
    constexpr float ki_q  = 1500.0f;

    // Speed controller (outer loop, 2 kHz)
    constexpr float kp_speed = 0.05f;
    constexpr float ki_speed = 0.05f;

    // Field weakening (outer loop, 2 kHz)
    constexpr float kp_fw            = 0.02f;
    constexpr float ki_fw            = 400.0f;
    constexpr float fw_voltage_target = 0.95f;  // fraction of v_max

    // Current limits
    constexpr float i_max = 60.0f;   // peak current (A)
}

namespace faults {
    constexpr float overspeed_rpm      = 20000.0f;  // mechanical speed (RPM)
    constexpr float overvoltage_v      = 560.0f;    // DC bus voltage (V)
    constexpr float overcurrent_a      = 75.0f;     // peak phase current (A)
    constexpr float motor_overtemp_c   = 140.0f;    // motor winding temperature (°C)
    constexpr float ambient_overtemp_c = 70.0f;     // ambient enclosure temperature (°C)
    constexpr float module_overtemp_c  = 100.0f;    // power module temperature (°C)
}

namespace encoder {
    // Total angle measurement latency: ADC sample + DMA transfer + ISR entry.
    // Set to 1.5x the inner loop period as a realistic default (75 µs at 20 kHz).
    constexpr float delay_s = 75e-6f;
}

} // namespace cobalt