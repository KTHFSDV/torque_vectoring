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
//
// SteeringCommand layout: [delta (rad), vx_ref (m/s), r_ref (rad/s)]
// Call setDt(dt) once per control cycle before computeImpl.
class PIDYawController : public IYawController<PIDYawController> {
   public:
    // Yaw PID params:
    //   kp_yaw        proportional gain on yaw-rate error
    //   ki_yaw        integral gain on yaw-rate error
    //   kd_yaw        derivative gain on the yaw-rate error
    //   i_max_yaw     symmetric saturation on the integral term |ki·∫e_r dt| [Nm]
    //   d_tau_yaw     low-pass time constant on the derivative term [s]
    //
    // Longitudinal 2-DoF PI params:
    //   mass          vehicle mass [kg]
    //   k_p           proportional gain [1/s]
    //   k_i           integral gain     [1/s²]
    //   b             setpoint weight on the proportional term [0, 1] (2-DoF PI)
    //   eta           drivetrain efficiency (0, 1]
    //   mu            tyre-road friction coefficient (for the Fx grip clamp)
    //   vx_ref_rate   max slew of the velocity reference [m/s²]
    //
    //   ep = b·vx_ref − vx     (setpoint-weighted proportional error)
    //   ev =   vx_ref − vx     (integral error — sees the full error)
    //   Fx_total = clamp((k_p·ep + k_i·∫ev dt) · mass / eta, ±μ·m·g)
    //   Errors are formed against a rate-limited vx_ref (slew ≤ vx_ref_rate).
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

    // Default: all parameters from config.hpp.
    PIDYawController()
        : PIDYawController(PIDYawControllerConfig::kp, PIDYawControllerConfig::ki,
                           PIDYawControllerConfig::kd, PIDYawControllerConfig::i_max,
                           PIDYawControllerConfig::d_tau, VehicleConfig::mass,
                           LongitudinalPiConfig::k_p, LongitudinalPiConfig::k_i,
                           LongitudinalPiConfig::b, LongitudinalPiConfig::eta, VehicleConfig::mu,
                           LongitudinalPiConfig::vx_ref_rate) {}

    // Must be called once per control cycle before computeImpl.
    void setDt(double dt) { dt_ = dt; }

    // VehicleState:    [vx, vy, yaw_rate, steering_angle, ax, ay]
    // SteeringCommand: uses steering_angle_, vx_ref_, r_ref_
    [[nodiscard]] YawMomentCommand computeImpl(const VehicleState& s, const ForceVector& forces,
                                               const SteeringCommand& cmd) {
        (void)forces;  // reserved for future combined-slip feedforward

        const double vx = s.vx_;
        const double r = s.yaw_rate_;
        const double vx_ref = cmd.vx_ref_;
        const double r_ref = cmd.r_ref_;

        // PID for yaw rate tracking.
        // Yaw-rate error is bidirectional (r_ref < 0 in right-hand turns), so the
        // integrator must accumulate in both directions. Anti-windup is a symmetric
        // clamp on the integral term rather than a one-sided reset — the previous
        // `if (e_r < 0) reset` dumped the integral on every right turn.
        const double e_r = r_ref - r;
        e_r_integral_ += e_r * dt_;
        if (ki_yaw_ > 0.0) {
            const double integral_limit = i_max_yaw_ / ki_yaw_;
            e_r_integral_ = std::clamp(e_r_integral_, -integral_limit, integral_limit);
        }

        // Derivative term on the yaw-rate error, low-pass filtered (time constant
        // d_tau_yaw). The error comes straight from the gyro, so the raw finite
        // difference is noise-dominated — kd on it would inject noise into Mz.
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

        // -----------------------------------------------------------------------
        // Fx demand — 2-DoF PI velocity tracker (no feedforward).
        //
        //   Fx_total = (k_p·ep + k_i·∫ev dt) · m / η
        // -----------------------------------------------------------------------
        // Rate-limit the velocity reference so a stepped target (e.g. launch from
        // rest) can't produce a huge instantaneous error that slams Fx to
        // saturation. Seed the ramp from the current speed on the first tick.
        if (!vx_ref_init_) {
            vx_ref_ramped_ = vx;
            vx_ref_init_ = true;
        }
        const double max_step = vx_ref_rate_ * dt_;
        vx_ref_ramped_ = std::clamp(vx_ref, vx_ref_ramped_ - max_step, vx_ref_ramped_ + max_step);

        // 2-DoF PI: the integral acts on the full error (drives steady-state to
        // zero), while the proportional acts on a setpoint-weighted error so a
        // reference step doesn't produce a full proportional kick.
        // Anti-windup is a symmetric clamp sized so the integral term alone stays
        // within the grip limit — the error is bidirectional (overspeed needs a
        // negative Fx), so a one-sided reset would dump the integral on every
        // overshoot of the ramped reference and preclude integral action while
        // decelerating.
        const double e_vx = vx_ref_ramped_ - vx;              // integral error
        const double e_vx_p = (b_sp_ * vx_ref_ramped_) - vx;  // proportional error
        e_vx_integral_ += e_vx * dt_;
        if (ki_longi_ > 0.0) {
            const double integral_limit = (mu_ * kGravity * eta_) / ki_longi_;
            e_vx_integral_ = std::clamp(e_vx_integral_, -integral_limit, integral_limit);
        }
        const double fx_raw =
            ((kp_longi_ * e_vx_p) + (ki_longi_ * e_vx_integral_)) * (mass_ / eta_);

        // Clamp to the longitudinal grip limit ±μ·m·g so the demand stays within
        // what the tyres can deliver (the allocator would otherwise ride saturation).
        const double fx_max = mu_ * mass_ * kGravity;
        const double fx_total = std::clamp(fx_raw, -fx_max, fx_max);
        return YawMomentCommand{.mz_ = mz, .fx_total_ = fx_total};
    }

   private:
    // PID Yaw parameters
    const double kp_yaw_;
    const double ki_yaw_;
    const double kd_yaw_;     // derivative gain (on yaw-rate error)
    const double i_max_yaw_;  // symmetric integral-term clamp [Nm]
    const double d_tau_yaw_;  // derivative low-pass time constant [s]

    // Longitudinal 2-DoF PI parameters
    const double mass_;         // vehicle mass [kg]
    const double kp_longi_;     // proportional gain [1/s]
    const double ki_longi_;     // integral gain     [1/s²]
    const double b_sp_;         // setpoint weight on the proportional term [0, 1]
    const double eta_;          // drivetrain efficiency
    const double mu_;           // tyre-road friction coefficient (Fx grip clamp)
    const double vx_ref_rate_;  // max velocity-reference slew [m/s²]

    // Yaw PID state
    double e_r_integral_{0.0};  // integral of yaw rate error [rad]
    double e_r_prev_{0.0};      // previous yaw-rate error [rad/s] (for the derivative term)
    double de_r_filt_{0.0};     // low-pass-filtered error derivative [rad/s²]
    bool yaw_d_init_{false};    // seed e_r_prev_ on the first tick to avoid a spike

    // PI state
    double e_vx_integral_{0.0};  // integral of velocity error [m]
    double vx_ref_ramped_{0.0};  // rate-limited velocity reference [m/s]
    bool vx_ref_init_{false};    // seed the ramp from measured vx on first tick

    // Time step — updated each cycle via setDt()
    double dt_{0.0};
};

}  // namespace tv
