#pragma once

#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

// High-level yaw moment controller — docs/tv.md §layer 2
//
//   Yaw moment: PID on the yaw-rate error (derivative low-pass filtered)
//   Fx demand : 2-DoF PI velocity tracker (TU Munich / KIT FSG approach)
class PIDYawController : public IYawController<PIDYawController> {
   public:
    PIDYawController(double kp_yaw, double ki_yaw, double kd_yaw, double i_max_yaw,
                     double d_tau_yaw, double mass, double k_p, double k_i, double b, double eta,
                     double mu, double vx_ref_rate)
        : kp_yaw_(kp_yaw),
          ki_yaw_(ki_yaw),
          kd_yaw_(kd_yaw),
          i_max_yaw_(i_max_yaw),
          d_tau_yaw_(d_tau_yaw),
          mass_(mass),
          kp_longi_(k_p),
          ki_longi_(k_i),
          b_sp_(b),
          eta_(eta),
          mu_(mu),
          vx_ref_rate_(vx_ref_rate) {}

    PIDYawController()
        : PIDYawController(PIDYawControllerConfig::kp, PIDYawControllerConfig::ki,
                           PIDYawControllerConfig::kd, PIDYawControllerConfig::i_max,
                           PIDYawControllerConfig::d_tau, VehicleConfig::mass,
                           LongitudinalPiConfig::k_p, LongitudinalPiConfig::k_i,
                           LongitudinalPiConfig::b, LongitudinalPiConfig::eta, VehicleConfig::mu,
                           LongitudinalPiConfig::vx_ref_rate) {}

    // Must be called once per control cycle before computeImpl.
    void setDt(double dt) { dt_ = dt; }

    [[nodiscard]] YawMomentCommand computeImpl(const VehicleState& s, const ForceVector& forces,
                                               const SteeringCommand& cmd) {
        (void)forces;  // reserved for future combined-slip feedforward

        const double vx = s.vx_;
        const double r = s.yaw_rate_;
        const double vx_ref = cmd.vx_ref_;
        const double r_ref = cmd.r_ref_;

        // The yaw-rate error is bidirectional (r_ref < 0 in right-hand turns), so
        // anti-windup must be a symmetric clamp, not a one-sided reset.
        const double e_r = r_ref - r;
        e_r_integral_ += e_r * dt_;
        if (ki_yaw_ > 0.0) {
            const double integral_limit = i_max_yaw_ / ki_yaw_;
            e_r_integral_ = std::clamp(e_r_integral_, -integral_limit, integral_limit);
        }

        // The error comes straight from the gyro, so the raw finite difference is
        // noise-dominated — low-pass it before applying kd.
        double mz_d = 0.0;
        if (yaw_d_init_ && dt_ > 0.0) {
            const double de_r_raw = (e_r - e_r_prev_) / dt_;
            const double beta = dt_ / (d_tau_yaw_ + dt_);
            de_r_filt_ += beta * (de_r_raw - de_r_filt_);
            mz_d = kd_yaw_ * de_r_filt_;
        }
        e_r_prev_ = e_r;
        yaw_d_init_ = true;

        const double mz = (kp_yaw_ * e_r) + (ki_yaw_ * e_r_integral_) + mz_d;

        // Rate-limit the velocity reference so a stepped target (e.g. launch from
        // rest) can't slam Fx to saturation; seed the ramp from the current speed.
        if (!vx_ref_init_) {
            vx_ref_ramped_ = vx;
            vx_ref_init_ = true;
        }
        const double max_step = vx_ref_rate_ * dt_;
        vx_ref_ramped_ = std::clamp(vx_ref, vx_ref_ramped_ - max_step, vx_ref_ramped_ + max_step);

        // 2-DoF PI: Fx_total = (k_p·ep + k_i·∫ev dt)·m/η. The integral sees the full
        // error; the proportional a setpoint-weighted one, so a reference step doesn't
        // produce a full proportional kick. The clamp is sized so the integral term
        // alone stays within the grip limit, and is symmetric because the error is
        // bidirectional (overspeed needs a negative Fx).
        const double e_vx = vx_ref_ramped_ - vx;              // integral error
        const double e_vx_p = (b_sp_ * vx_ref_ramped_) - vx;  // proportional error
        e_vx_integral_ += e_vx * dt_;
        if (ki_longi_ > 0.0) {
            const double integral_limit = (mu_ * kGravity * eta_) / ki_longi_;
            e_vx_integral_ = std::clamp(e_vx_integral_, -integral_limit, integral_limit);
        }
        const double fx_raw =
            ((kp_longi_ * e_vx_p) + (ki_longi_ * e_vx_integral_)) * (mass_ / eta_);

        // Keep the demand inside the longitudinal grip limit ±μ·m·g.
        const double fx_max = mu_ * mass_ * kGravity;
        const double fx_total = std::clamp(fx_raw, -fx_max, fx_max);
        return YawMomentCommand{.mz_ = mz, .fx_total_ = fx_total};
    }

   private:
    const double kp_yaw_;
    const double ki_yaw_;
    const double kd_yaw_;
    const double i_max_yaw_;  // symmetric integral-term clamp [Nm]
    const double d_tau_yaw_;  // derivative low-pass time constant [s]

    const double mass_;         // vehicle mass [kg]
    const double kp_longi_;     // [1/s]
    const double ki_longi_;     // [1/s²]
    const double b_sp_;         // setpoint weight on the proportional term [0, 1]
    const double eta_;          // drivetrain efficiency
    const double mu_;           // tyre-road friction coefficient
    const double vx_ref_rate_;  // max velocity-reference slew [m/s²]

    double e_r_integral_{0.0};  // [rad]
    double e_r_prev_{0.0};      // [rad/s]
    double de_r_filt_{0.0};     // [rad/s²]
    bool yaw_d_init_{false};

    double e_vx_integral_{0.0};  // [m]
    double vx_ref_ramped_{0.0};  // [m/s]
    bool vx_ref_init_{false};

    double dt_{0.0};  // control period [s], set each cycle via setDt()
};

}  // namespace tv
