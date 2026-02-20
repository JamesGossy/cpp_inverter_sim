#pragma once
#include "Vec.hpp"
#include "PIController.hpp"

namespace olinv {

struct CurrentControlParams {
    double R{0.05};
    double Ld{200e-6};
    double Lq{200e-6};
    double psi_f{0.015};  // [Wb]
    double Vdc{48.0};     // [V]
};

class CurrentController {
public:
    explicit CurrentController(const CurrentControlParams& p);

    void reset();
    void set_pi_gains(double kp_d, double ki_d, double kp_q, double ki_q);

    // Returns commanded voltage in dq frame (v_d, v_q)
    Vec2 step(const Vec2& i_dq_ref,
              const Vec2& i_dq_meas,
              double omega_e, // electrical rad/s
              double dt);

private:
    CurrentControlParams p_;
    PIController pi_d_;
    PIController pi_q_;

    // dq voltage magnitude limit (simple SPWM-friendly limit)
    Vec2 limit_vdq_spwm(const Vec2& v_dq) const;
};

} // namespace olinv