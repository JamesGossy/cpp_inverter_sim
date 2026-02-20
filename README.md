# Open-loop inverter + dq motor model + FOC transforms + dq current PI (simulation)

Minimal C++ simulation of:
- Clarke/Park transforms (FOC)
- dq PI current controller with decoupling/feedforward
- PMSM dq electrical + mechanical model (RK4 integration)
- Average-value 3-phase inverter (floating neutral)
- SPWM duty computation (equivalent to comparing sine vs triangle)
- Multi-rate loop: control/PWM at 20 kHz, motor integration with substeps
- CSV logging + Python plotting script

## Build
```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j# cpp_inverter_sim