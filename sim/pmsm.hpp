// pmsm.hpp
// Simulates a permanent magnet motor: tracks currents, speed, and position over time

#pragma once
#include <cmath>

namespace sim {

const double PI     = 3.14159265358979323846;
const double TWO_PI = 2.0 * PI;

class PMSM {
public:

    // All the physical properties of the motor
    struct Params {
        int    pole_pairs;  // number of magnetic pole pairs
        double Rs;          // winding resistance (ohms)
        double Ld;          // d-axis inductance (henries)
        double Lq;          // q-axis inductance (henries)
        double psi_f;       // flux from the permanent magnets
        double J;           // rotor inertia (kg·m²)
        double B;           // friction/damping coefficient
        double T_load;      // constant load torque applied to the shaft
    };

    // Set up the motor with the given parameters
    PMSM(Params p) {
        params = p;
    }

    // Advance the motor simulation by one time step
    void step(double dt, double v_alpha, double v_beta);

    // Get the current electrical angle
    double theta_e() const {
        return std::fmod(params.pole_pairs * theta_m, TWO_PI);
    }

    // Get the current mechanical speed (rad/s)
    double omega_m() const { return speed; }

    // Get the current torque being produced
    double torque() const { return Te; }

    // Get the d and q axis currents
    double id() const { return id_; }
    double iq() const { return iq_; }

    // Convert d/q currents back to alpha/beta 
    void currents_alphabeta(double& i_alpha, double& i_beta) const;

private:
    Params params;
    double id_    = 0.0;  // d-axis current
    double iq_    = 0.0;  // q-axis current
    double speed  = 0.0;  // mechanical speed (rad/s)
    double theta_m = 0.0; // mechanical angle (radians)
    double Te     = 0.0;  // electromagnetic torque
};

}