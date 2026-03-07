// pmsm.hpp
// PMSM motor model in the dq frame.
// Takes applied phase voltages (alpha/beta) and steps electrical + mechanical state.

#pragma once
#include <cmath>

namespace sim {

static constexpr double kPi    = 3.14159265358979323846;
static constexpr double kTwoPi = 2.0 * kPi;

class PMSM {
public:
    struct Params {
        int    pole_pairs;
        double Rs;
        double Ld;
        double Lq;
        double psi_f;
        double J;
        double B;
        double T_load;
    };
    PMSM(const Params& p) : p_(p) {}

    void step(double dt_s, double v_alpha, double v_beta);

    double theta_e() const { return std::fmod(p_.pole_pairs * theta_m_, kTwoPi); }
    double omega_m() const { return omega_m_; }
    double torque()  const { return Te_; }
    double id()      const { return id_; }
    double iq()      const { return iq_; }

    // dq currents -> alpha/beta for logging
    void currents_alphabeta(double& i_alpha, double& i_beta) const;

private:
    Params p_;
    double id_{0.0}, iq_{0.0};
    double omega_m_{0.0}, theta_m_{0.0};
    double Te_{0.0};
};

}