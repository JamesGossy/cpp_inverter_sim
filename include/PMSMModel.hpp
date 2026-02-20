#pragma once
#include "Vec.hpp"

namespace olinv {

struct PMSMParams {
    // Electrical
    double R{0.05};
    double Ld{200e-6};
    double Lq{200e-6};
    double psi_f{0.015};  // [Wb]
    int pole_pairs{7};

    // Mechanical
    double J{1e-4};       // [kg*m^2]
    double B{1e-4};       // [N*m*s]
    double T_load{0.0};   // [N*m]
};

struct PMSMState {
    Vec2 i_dq{0.0, 0.0};  // [A]
    double omega_m{0.0};  // mech rad/s
    double theta_e{0.0};  // electrical rad
};

class PMSMModel {
public:
    explicit PMSMModel(const PMSMParams& p) : p_(p) {}

    const PMSMParams& params() const { return p_; }
    const PMSMState& state() const { return x_; }
    PMSMState& state() { return x_; }

    void reset(const PMSMState& x0);
    void step(const Vec2& v_dq, double dt);

    double omega_e() const { return static_cast<double>(p_.pole_pairs) * x_.omega_m; }
    double torque_e() const;

private:
    PMSMParams p_{};
    PMSMState x_{};

    struct Deriv {
        Vec2 di_dq;
        double domega_m;
        double dtheta_e;
    };

    Deriv deriv(const PMSMState& x, const Vec2& v_dq) const;
};

} // namespace olinv