#pragma once

#include <fs_msgs/msg/car_command.h>

#include <cstddef>
#include <numbers>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "config.hpp"
#include "fs_msgs/msg/car_command.hpp"
#include "fs_msgs/msg/wheelspeeds.hpp"
#include "orchestrator.hpp"
#include "ros2can_msgs/msg/dv_control_target_tv.hpp"
#include "ros2can_msgs/msg/vehicle_status_motor_torques.hpp"
#include "types.hpp"

namespace tv {

/// ROS2 node that wraps the full torque-vectoring pipeline.
///
/// Subscriptions:
///   imu             sensor_msgs/Imu                       — yaw_rate, ax, ay
///   odom            nav_msgs/Odometry                     — body velocities vx, vy
///   wheel_speeds    fs_msgs/Wheelspeeds                   — per-wheel ω [FL,FR,RL,RR] (rad/s)
///   motor_torques   ros2can_msgs/VehicleStatusMotorTorques — per-wheel motor torque (Nm)
///   cmd_in          fs_msgs/CarCommand                    — steering_angle [deg], vx_ref, r_ref
///
/// Publication:
///   cmd_out         ros2can_msgs/DvControlTargetTv        — per-wheel targets + steering
///
/// The pipeline is triggered on every IMU message. Odometry, wheel-speed,
/// motor-torque and steering updates are cached on receipt and take effect on
/// the next IMU tick. Wheel-speed messages carry their own sample period, used
/// by the DFO force estimator.
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
class TVNode : public rclcpp::Node {
   public:
    explicit TVNode(Orchestrator<F, Y, A> orchestrator,
                    const std::string& node_name = "torque_vectoring")
        : Node(node_name), orchestrator_(std::move(orchestrator)) {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

        wheel_speeds_sub_ = create_subscription<fs_msgs::msg::Wheelspeeds>(
            "wheel_speeds", rclcpp::SensorDataQoS(),
            [this](fs_msgs::msg::Wheelspeeds::SharedPtr msg) { wheelSpeedsCallback(msg); });

        motor_torques_sub_ = create_subscription<ros2can_msgs::msg::VehicleStatusMotorTorques>(
            "motor_torques", 10,
            [this](ros2can_msgs::msg::VehicleStatusMotorTorques::SharedPtr msg) {
                motorTorquesCallback(msg);
            });

        steering_sub_ = create_subscription<fs_msgs::msg::CarCommand>(
            "cmd_in", 10,
            [this](fs_msgs::msg::CarCommand::SharedPtr msg) { steeringCallback(msg); });

        torque_pub_ = create_publisher<ros2can_msgs::msg::DvControlTargetTv>("cmd_out", 10);

        // Debug publishers for every intermediate pipeline signal. These carry no
        // control meaning — they exist for telemetry/plotting (rqt_plot, PlotJuggler).
        // Wheel order for the array topics is [FL, FR, RL, RR].
        tire_fx_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug/tire_fx", 10);
        tire_fy_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug/tire_fy", 10);
        tire_fz_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug/tire_fz", 10);
        yaw_moment_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>("debug/yaw_moment", 10);
        wheel_torques_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>("debug/wheel_torques", 10);
        timing_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug/timing", 10);
        tracking_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("debug/tracking", 10);

#ifdef DEBUG
        // Achieved virtual demand *after* the QP allocation, reconstructed from the
        // allocated wheel torques. Divergence from debug/yaw_moment (the requested
        // demand) shows how far the box/friction constraints pushed the solution off
        // the request.
        alloc_yaw_moment_pub_ =
            create_publisher<std_msgs::msg::Float64>("debug/alloc_yaw_moment", 10);
        alloc_fx_total_pub_ = create_publisher<std_msgs::msg::Float64>("debug/alloc_fx_total", 10);
#endif
    }

   private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg) {
        state_.yaw_rate_ = msg->angular_velocity.z;  // yaw_rate  [rad/s]
        state_.ax_ = msg->linear_acceleration.x;     // ax        [m/s²]
        state_.ay_ = msg->linear_acceleration.y;     // ay        [m/s²]

        // Control period since the previous IMU tick, used by the stage
        // integrators. Guarded so the first tick (and any out-of-order or
        // duplicate stamps) yields dt = 0 rather than a bogus/negative step.
        const rclcpp::Time stamp(msg->header.stamp);
        double dt = 0.0;
        if (have_last_stamp_ && stamp > last_stamp_) {
            dt = (stamp - last_stamp_).seconds();
        }
        last_stamp_ = stamp;
        have_last_stamp_ = true;

        // Input watchdog: if driver commands or odometry are stale (or were never
        // received), command zero torque instead of acting on old references.
        const rclcpp::Time now_time = now();
        const auto is_stale = [&](bool received, const rclcpp::Time& last_rx) {
            return !received || (now_time - last_rx).seconds() > NodeConfig::input_timeout;
        };
        if (is_stale(have_cmd_, last_cmd_rx_) || is_stale(have_odom_, last_odom_rx_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "cmd_in/odom stale or missing — commanding zero torque");
            ros2can_msgs::msg::DvControlTargetTv out;  // torque targets default to zero
            out.dv_steering_angle_target_tv =
                steering_cmd_.steering_angle_ * (180.0 / std::numbers::pi);
            torque_pub_->publish(out);
            return;
        }

        const PipelineResult result = orchestrator_.run(state_, steering_cmd_, dt);
        const WheelTorques& torques = result.torques;

        // The allocator holds its previous solution on a failed solve — surface it.
        if (!result.alloc_ok) {
            ++alloc_failures_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "torque allocation failed (%lu total) — holding previous solution",
                static_cast<unsigned long>(alloc_failures_));
        }

        ros2can_msgs::msg::DvControlTargetTv out;
        // steering_angle_ is stored in radians internally; the CAN target expects degrees.
        out.dv_steering_angle_target_tv = steering_cmd_.steering_angle_ * (180.0 / std::numbers::pi);
        out.dv_fl_speedd_target = torques[0];
        out.dv_fr_speed_target = torques[1];
        out.dv_rl_speed_target = torques[2];
        out.dv_rr_speed_target = torques[3];
        torque_pub_->publish(out);

        publishDebug(result);
    }

    // Publishes each intermediate pipeline stage on its own debug topic.
    void publishDebug(const PipelineResult& result) {
        const auto to_msg = [](const std::array<double, 4>& src) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.assign(src.begin(), src.end());
            return msg;
        };

        // Stage 1 — tyre force estimate (per wheel).
        tire_fx_pub_->publish(to_msg(result.forces.fx_));
        tire_fy_pub_->publish(to_msg(result.forces.fy_));
        tire_fz_pub_->publish(to_msg(result.forces.fz_));

        // Stage 2 — high-level yaw moment command: [mz (Nm), fx_total (N)].
        std_msgs::msg::Float64MultiArray yaw_msg;
        yaw_msg.data = {result.yaw_cmd.mz_, result.yaw_cmd.fx_total_};
        yaw_moment_pub_->publish(yaw_msg);

        // Stage 3 — allocated wheel torques [FL, FR, RL, RR] (Nm).
        std_msgs::msg::Float64MultiArray torque_msg;
        torque_msg.data = {result.torques[0], result.torques[1], result.torques[2],
                           result.torques[3]};
        wheel_torques_pub_->publish(torque_msg);

        // Per-stage pipeline execution times (µs), three blocks of
        // [forces, yaw, alloc, total]: this cycle, running mean, and running
        // standard deviation since startup.
        std_msgs::msg::Float64MultiArray timing_msg;
        timing_msg.data = {result.timings.forces_us,     result.timings.yaw_us,
                           result.timings.alloc_us,      result.timings.total_us,
                           result.timings_avg.forces_us, result.timings_avg.yaw_us,
                           result.timings_avg.alloc_us,  result.timings_avg.total_us,
                           result.timings_std.forces_us, result.timings_std.yaw_us,
                           result.timings_std.alloc_us,  result.timings_std.total_us};
        timing_pub_->publish(timing_msg);

        // Controller tracking: [r_ref, r, r_ref − r, vx_ref, vx, vx_ref − vx].
        // A yaw-rate error that never changes sign (or r_ref/r with opposite
        // signs in a corner) points at a reference sign/unit mismatch or the
        // yaw integral riding its clamp — the usual causes of a standing
        // left/right torque split.
        std_msgs::msg::Float64MultiArray tracking_msg;
        tracking_msg.data = {steering_cmd_.r_ref_,
                             state_.yaw_rate_,
                             steering_cmd_.r_ref_ - state_.yaw_rate_,
                             steering_cmd_.vx_ref_,
                             state_.vx_,
                             steering_cmd_.vx_ref_ - state_.vx_};
        tracking_pub_->publish(tracking_msg);

#ifdef DEBUG
        // Achieved [Mz, Fx_total] after allocation. Undo the allocator's output map
        // (T_motor = F·R / gear_ratio) to recover the per-wheel driving force
        // F = T·gear_ratio / R, then apply the dependency matrix G (wheel order
        // [FL, FR, RL, RR]):
        //   Fx_total =            F_FL + F_FR + F_RL + F_RR
        //   Mz       = -sf·F_FL + sf·F_FR - sr·F_RL + sr·F_RR
        const double sf = VehicleConfig::trackwidth_front / 2.0;
        const double sr = VehicleConfig::trackwidth_rear / 2.0;
        std::array<double, 4> f{};  // per-wheel driving force [N], [FL, FR, RL, RR]
        for (int i = 0; i < 4; ++i) {
            f[i] = result.torques[i] * VehicleConfig::gear_ratio / VehicleConfig::wheel_radius;
        }

        std_msgs::msg::Float64 alloc_fx_msg;
        alloc_fx_msg.data = f[0] + f[1] + f[2] + f[3];
        alloc_fx_total_pub_->publish(alloc_fx_msg);

        std_msgs::msg::Float64 alloc_mz_msg;
        alloc_mz_msg.data = (-sf * f[0]) + (sf * f[1]) + (-sr * f[2]) + (sr * f[3]);
        alloc_yaw_moment_pub_->publish(alloc_mz_msg);
#endif
    }

    void steeringCallback(const fs_msgs::msg::CarCommand::SharedPtr& msg) {
        // CarCommand.steering is a road-wheel angle in degrees; the pipeline works
        // in radians throughout, so convert on ingress.
        const double steering_rad = msg->steering * (std::numbers::pi / 180.0);

        steering_cmd_.steering_angle_ = steering_rad;  // steering_angle [rad]
        steering_cmd_.vx_ref_ = msg->velocity;         // vx_ref         [m/s]
        steering_cmd_.r_ref_ = msg->yaw_rate;          // r_ref          [rad/s]
        state_.steering_angle_ = steering_rad;         // mirror into VehicleState

        last_cmd_rx_ = now();
        have_cmd_ = true;
    }

    // Body-frame velocities from odometry; consumed on the next IMU tick.
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
        state_.vx_ = msg->twist.twist.linear.x;  // vx [m/s]
        state_.vy_ = msg->twist.twist.linear.y;  // vy [m/s]

        last_odom_rx_ = now();
        have_odom_ = true;
    }

    // Per-wheel angular velocities [FL, FR, RL, RR] (rad/s) for the DFO. The
    // sample period is derived from consecutive wheel-speed stamps and forwarded
    // to the force estimator together with the latest motor-torque feedback.
    void wheelSpeedsCallback(const fs_msgs::msg::Wheelspeeds::SharedPtr& msg) {
        WheelOmegas omegas;
        omegas << msg->front_left, msg->front_right, msg->rear_left, msg->rear_right;

        const rclcpp::Time stamp(msg->header.stamp);
        double dt = 0.0;
        if (have_last_wheel_stamp_ && stamp > last_wheel_stamp_) {
            dt = (stamp - last_wheel_stamp_).seconds();
        }
        last_wheel_stamp_ = stamp;
        have_last_wheel_stamp_ = true;

        orchestrator_.updateWheelState(motor_torques_, omegas, dt);
    }

    // Latest per-wheel motor-torque feedback [FL, FR, RL, RR] (Nm), cached for
    // the DFO (published at a slower rate than the wheel speeds).
    void motorTorquesCallback(const ros2can_msgs::msg::VehicleStatusMotorTorques::SharedPtr& msg) {
        motor_torques_ << msg->torque_fl, msg->torque_fr, msg->torque_rl, msg->torque_rr;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<fs_msgs::msg::Wheelspeeds>::SharedPtr wheel_speeds_sub_;
    rclcpp::Subscription<ros2can_msgs::msg::VehicleStatusMotorTorques>::SharedPtr motor_torques_sub_;
    rclcpp::Subscription<fs_msgs::msg::CarCommand>::SharedPtr steering_sub_;
    rclcpp::Publisher<ros2can_msgs::msg::DvControlTargetTv>::SharedPtr torque_pub_;

    // Debug publishers for intermediate pipeline stages.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tire_fx_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tire_fy_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tire_fz_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr yaw_moment_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_torques_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr timing_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tracking_pub_;

#ifdef DEBUG
    // Achieved yaw moment / total Fx reconstructed from the allocated torques.
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alloc_yaw_moment_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr alloc_fx_total_pub_;
#endif

    Orchestrator<F, Y, A> orchestrator_;
    VehicleState state_{};            // vx_, vy_, yaw_rate_, steering_angle_, ax_, ay_
    SteeringCommand steering_cmd_{};  // steering_angle_, vx_ref_, r_ref_

    // Latest per-wheel motor-torque feedback [FL, FR, RL, RR] (Nm) for the DFO.
    WheelTorques motor_torques_{WheelTorques::Zero()};

    // Timestamp of the previous IMU message, used to derive the control period.
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    bool have_last_stamp_{false};

    // Timestamp of the previous wheel-speed message, used to derive the DFO
    // sample period.
    rclcpp::Time last_wheel_stamp_{0, 0, RCL_ROS_TIME};
    bool have_last_wheel_stamp_{false};

    // Receive times of the latest cmd_in / odom messages (node clock), for the
    // input watchdog — stale inputs make the node command zero torque.
    rclcpp::Time last_cmd_rx_{0, 0, RCL_ROS_TIME};
    bool have_cmd_{false};
    rclcpp::Time last_odom_rx_{0, 0, RCL_ROS_TIME};
    bool have_odom_{false};

    // Count of control cycles where the allocator fell back to zero torque.
    std::size_t alloc_failures_{0};
};

// CTAD deduction guides
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
TVNode(Orchestrator<F, Y, A>) -> TVNode<F, Y, A>;

template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
TVNode(Orchestrator<F, Y, A>, const std::string&) -> TVNode<F, Y, A>;

}  // namespace tv
