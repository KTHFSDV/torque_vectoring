#pragma once

#include <algorithm>

#include <Eigen/Dense>

#include "config.hpp"
#include "interfaces.hpp"
#include "types.hpp"

namespace tv {

class KinematicForceEstimator : public IForceEstimator<KinematicForceEstimator> {
   public:
    KinematicForceEstimator()
        : mass_(VehicleConfig::mass),
          cg_height_(VehicleConfig::cg_height),
          wheelbase_(VehicleConfig::wheelbase),
          l_r_(VehicleConfig::l_r),
          trackwidth_(VehicleConfig::trackwidth_front),
          front_tlltd_(VehicleConfig::front_tlltd),
          j_w_(ForceEstimatorConfig::j_w),
          r_eff_(VehicleConfig::wheel_radius),
          gear_ratio_(VehicleConfig::gear_ratio),
          eta_(ForceEstimatorConfig::eta),
          lpf_tau_(ForceEstimatorConfig::lpf_tau) {}

    // Driving Force Observer, from the wheel-axle torque balance
    // J_w·dω/dt = T_wheel − F_x·R_eff:
    //   F̂_x = (T_wheel − J_w·dω/dt) / R_eff,   T_wheel = N·η·T_motor
    // The inverter reports motor-side torque, hence the conversion to the wheel
    // axle, where ω, J_w and R_eff live. Wheel order [FL, FR, RL, RR]; dt is the
    // wheel-speed sample period [s] — the DFO runs here rather than at control
    // rate so the LPF honours lpf_tau whatever the IMU rate is.
    // Reference: Hori (2004), adopted by FS Team Tallinn (world #1 FSE 2024).
    void updateWheelState(const WheelTorques& motor_torques, const WheelOmegas& wheel_omegas,
                          double dt) {
        // Seed on the first sample so dω/dt doesn't spike at startup.
        if (!wheel_init_) {
            current_omegas_ = wheel_omegas;
            wheel_init_ = true;
        }
        prev_omegas_ = current_omegas_;
        current_omegas_ = wheel_omegas;
        motor_torques_ = motor_torques;

        if (dt <= 0.0) {
            return;
        }
        const double alpha = lpf_tau_ / (lpf_tau_ + dt);
        for (int i = 0; i < 4; ++i) {
            const double d_omega_dt = (current_omegas_[i] - prev_omegas_[i]) / dt;
            const double t_wheel = gear_ratio_ * eta_ * motor_torques_[i];
            const double fx_raw = (t_wheel - j_w_ * d_omega_dt) / r_eff_;
            fx_filtered_[i] = alpha * fx_filtered_[i] + (1.0 - alpha) * fx_raw;
        }
    }

    [[nodiscard]] ForceVector estimateForcesImpl(const VehicleState& s) {
        const double ax = s.ax_;
        const double ay = s.ay_;
        const double l_f = wheelbase_ - l_r_;

        ForceVector forces{};

        for (int i = 0; i < 4; ++i) forces.fx_[i] = fx_filtered_[i];

        // Quasi-static load transfer from IMU ax, ay — no kinematic approximation.
        // Reference: AMZ Racing (ETH Zurich), docs/tire_forces.md §1.
        const double fz_front_static = 0.5 * mass_ * kGravity * l_r_ / wheelbase_;
        const double fz_rear_static = 0.5 * mass_ * kGravity * l_f / wheelbase_;

        const double delta_fz_lat = mass_ * ay * cg_height_ / trackwidth_;
        const double delta_fz_lat_front = front_tlltd_ * delta_fz_lat;
        const double delta_fz_lat_rear = (1.0 - front_tlltd_) * delta_fz_lat;
        const double delta_fz_long = 0.5 * mass_ * ax * cg_height_ / wheelbase_;

        forces.fz_[0] = fz_front_static - delta_fz_long - delta_fz_lat_front;  // FL
        forces.fz_[1] = fz_front_static - delta_fz_long + delta_fz_lat_front;  // FR
        forces.fz_[2] = fz_rear_static + delta_fz_long - delta_fz_lat_rear;    // RL
        forces.fz_[3] = fz_rear_static + delta_fz_long + delta_fz_lat_rear;    // RR

        // Steady-state bicycle model. The axle force is split by each wheel's share of
        // the axle load, not 50/50: an even split overstates the unloaded inner wheel's
        // Fy, collapsing its friction-circle budget in the allocator to zero.
        const double fy_front_axle = (l_r_ / wheelbase_) * mass_ * ay;
        const double fy_rear_axle = (l_f / wheelbase_) * mass_ * ay;

        // Left wheel's share of the axle load; 0.5 when the axle is unloaded.
        const auto left_share = [](double fz_left, double fz_right) {
            const double left = std::max(0.0, fz_left);
            const double sum = left + std::max(0.0, fz_right);
            return sum > 1e-6 ? left / sum : 0.5;
        };
        const double front_left_share = left_share(forces.fz_[0], forces.fz_[1]);
        const double rear_left_share = left_share(forces.fz_[2], forces.fz_[3]);

        forces.fy_[0] = fy_front_axle * front_left_share;          // FL
        forces.fy_[1] = fy_front_axle * (1.0 - front_left_share);  // FR
        forces.fy_[2] = fy_rear_axle * rear_left_share;            // RL
        forces.fy_[3] = fy_rear_axle * (1.0 - rear_left_share);    // RR

        return forces;
    }

   private:
    double mass_;
    double cg_height_;
    double wheelbase_;
    double l_r_;
    double trackwidth_;
    double front_tlltd_;  // front TLLTD fraction, e.g. 0.52

    double j_w_;         // rotational inertia referred to the wheel [kg·m²]
    double r_eff_;       // effective wheel radius [m]
    double gear_ratio_;  // motor → wheel gear ratio
    double eta_;         // drivetrain efficiency (motor → wheel)
    double lpf_tau_;     // LPF time constant [s]

    WheelOmegas prev_omegas_{WheelOmegas::Zero()};
    WheelOmegas current_omegas_{WheelOmegas::Zero()};
    WheelTorques motor_torques_{WheelTorques::Zero()};
    Eigen::Vector4d fx_filtered_{Eigen::Vector4d::Zero()};
    bool wheel_init_{false};
};

}  // namespace tv
