// system/faults.hpp
// Latching fault flags for the Cobalt motor drive.
// Call updateFaults() each tick; gate the inverter on !faults.any().

#pragma once
#include "config/cobalt_params.hpp"

namespace cobalt {

struct Faults {
    bool overspeed        = false;
    bool overvoltage      = false;
    bool overcurrent      = false;
    bool motor_overtemp   = false;
    bool ambient_overtemp = false;
    bool module_overtemp  = false;

    bool any() const {
        return overspeed | overvoltage | overcurrent
             | motor_overtemp | ambient_overtemp | module_overtemp;
    }

    void clear() {
        overspeed = overvoltage = overcurrent
            = motor_overtemp = ambient_overtemp = module_overtemp = false;
    }
};

// speed_rpm and current_a are always available from the sim/sensors.
// Temp inputs default to 0 so the sim can omit them until thermal modelling is added.
inline void updateFaults(Faults& f,
                         float speed_rpm,
                         float vdc_v,
                         float current_a,
                         float motor_temp_c   = 0.0f,
                         float ambient_temp_c = 0.0f,
                         float module_temp_c  = 0.0f)
{
    namespace lim = cobalt::faults;
    if (speed_rpm      > lim::overspeed_rpm)      f.overspeed        = true;
    if (vdc_v          > lim::overvoltage_v)      f.overvoltage      = true;
    if (current_a      > lim::overcurrent_a)      f.overcurrent      = true;
    if (motor_temp_c   > lim::motor_overtemp_c)   f.motor_overtemp   = true;
    if (ambient_temp_c > lim::ambient_overtemp_c) f.ambient_overtemp = true;
    if (module_temp_c  > lim::module_overtemp_c)  f.module_overtemp  = true;
}

} // namespace cobalt