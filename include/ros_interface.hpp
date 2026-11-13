#pragma once

#include <fs_msgs/msg/car_command.h>

#include <Eigen/Geometry>
#include <cstddef>
#include <memory>
#include <optional>
#include <nav_msgs/msg/odometry.hpp>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "config.hpp"
#include "fs_msgs/msg/car_command.hpp"
#include "orchestrator.hpp"
#include "ros2can_msgs/msg/dv_control_target_tv.hpp"
#include "ros2can_msgs/msg/vehicle_status_motor_torques.hpp"
#include "ros2can_msgs/msg/vehicle_status_wheel_speed.hpp"
#include "types.hpp"

namespace tv {

/// ROS2 node that wraps the full torque-vectoring pipeline.
///
/// Subscriptions:
///   imu             sensor_msgs/Imu                       — yaw_rate, ax, ay (→ base_frame)
///   odom            nav_msgs/Odometry                     — body velocities vx, vy
///   wheel_speeds    ros2can_msgs/VehicleStatusWheelSpeed  — per-wheel motor rpm [FL,FR,RL,RR]
///   motor_torques   ros2can_msgs/VehicleStatusMotorTorques — per-wheel motor torque (Nm)
///   cmd_in          fs_msgs/CarCommand                    — steering_angle [deg], vx_ref, r_ref
///
/// Publication:
///   cmd_out         ros2can_msgs/DvControlTargetTv        — per-wheel targets + steering
///
/// The pipeline runs on every IMU message; the other inputs are cached on receipt
/// and take effect on the next IMU tick. Zero torque is commanded until the IMU →
/// base_frame transform is available.
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
class TVNode : public rclcpp::Node {
   public:
    explicit TVNode(Orchestrator<F, Y, A> orchestrator,
                    const std::string& node_name = "torque_vectoring")
        : Node(node_name), orchestrator_(std::move(orchestrator)) {
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCallback(msg); });

        wheel_speeds_sub_ = create_subscription<ros2can_msgs::msg::VehicleStatusWheelSpeed>(
            "wheel_speeds", 10,
            [this](ros2can_msgs::msg::VehicleStatusWheelSpeed::SharedPtr msg) {
                wheelSpeedsCallback(msg);
            });

        motor_torques_sub_ = create_subscription<ros2can_msgs::msg::VehicleStatusMotorTorques>(
            "motor_torques", 10,
            [this](ros2can_msgs::msg::VehicleStatusMotorTorques::SharedPtr msg) {
                motorTorquesCallback(msg);
            });

        steering_sub_ = create_subscription<fs_msgs::msg::CarCommand>(
            "cmd_in", 10,
            [this](fs_msgs::msg::CarCommand::SharedPtr msg) { steeringCallback(msg); });

        torque_pub_ = create_publisher<ros2can_msgs::msg::DvControlTargetTv>("cmd_out", 10);

        // Debug topics — telemetry only; wheel order [FL, FR, RL, RR].
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
        // Achieved virtual demand after allocation, reconstructed from the allocated
        // torques. Divergence from debug/yaw_moment shows how far the box/friction
        // constraints pushed the solution off the request.
        alloc_yaw_moment_pub_ =
            create_publisher<std_msgs::msg::Float64>("debug/alloc_yaw_moment", 10);
        alloc_fx_total_pub_ = create_publisher<std_msgs::msg::Float64>("debug/alloc_fx_total", 10);
#endif
    }

   private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg) {
        const std::optional<Eigen::Isometry3d> imu_to_base =
            lookupImuTransform(msg->header.frame_id);
        if (!imu_to_base) {
            publishZeroTorque();
            return;
        }

        // dt = 0 on the first tick and on out-of-order or duplicate stamps.
        const rclcpp::Time stamp(msg->header.stamp);
        double dt = 0.0;
        if (have_last_stamp_ && stamp > last_stamp_) {
            dt = (stamp - last_stamp_).seconds();
        }
        last_stamp_ = stamp;
        have_last_stamp_ = true;

        const Eigen::Matrix3d rot = imu_to_base->rotation();
        const Eigen::Vector3d omega =
            rot * Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                                  msg->angular_velocity.z);
        const Eigen::Vector3d accel_imu =
            rot * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                                  msg->linear_acceleration.z);

        // Lever-arm correction: a_base = a_imu - alpha x r - omega x (omega x r), with r the IMU
        // position in base_frame. alpha is a finite difference of omega; zero when dt is unknown.
        const Eigen::Vector3d alpha =
            (dt > 0.0 && have_last_omega_) ? Eigen::Vector3d((omega - last_omega_) / dt)
                                           : Eigen::Vector3d::Zero();
        last_omega_ = omega;
        have_last_omega_ = true;
        const Eigen::Vector3d r = imu_to_base->translation();
        const Eigen::Vector3d accel = accel_imu - alpha.cross(r) - omega.cross(omega.cross(r));

        if (first_imu_recieved_) {
            // 200 ms imu filter, assumes imu at 100 hz
            state_.yaw_rate_ = omega.z();
            state_.ax_ -= state_.ax_ / 20;
            state_.ax_ += accel.x() / 20;
            state_.ay_ -= state_.ay_ / 20;
            state_.ay_ += accel.y() / 20;
        } else {
            first_imu_recieved_ = true;
            state_.yaw_rate_ = omega.z();
            state_.ax_ = accel.x();
            state_.ay_ = accel.y();
        }

        // Stale or missing driver commands / odometry — command zero torque.
        const rclcpp::Time now_time = now();
        const auto is_stale = [&](bool received, const rclcpp::Time& last_rx) {
            return !received || (now_time - last_rx).seconds() > NodeConfig::input_timeout;
        };
        if (is_stale(have_cmd_, last_cmd_rx_) || is_stale(have_odom_, last_odom_rx_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "cmd_in/odom stale or missing — commanding zero torque");
            publishZeroTorque();
            return;
        }

        const PipelineResult result = orchestrator_.run(state_, steering_cmd_, dt);
        const WheelTorques& torques = result.torques;

        // The allocator holds its previous solution on a failed solve — surface it.
        if (!result.alloc_ok) {
            ++alloc_failures_;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "torque allocation failed (%lu total) — holding previous solution",
                                 static_cast<unsigned long>(alloc_failures_));
        }

        ros2can_msgs::msg::DvControlTargetTv out;
        // steering_angle_ is stored in radians internally; the CAN target expects degrees.
        out.dv_steering_angle_target_tv =
            steering_cmd_.steering_angle_ * (180.0 / std::numbers::pi);
        out.dv_fl_speedd_target = torques[0];
        out.dv_fr_speed_target = torques[1];
        out.dv_rl_speed_target = torques[2];
        out.dv_rr_speed_target = torques[3];
        torque_pub_->publish(out);

        publishDebug(result);
    }

    // The IMU mount is static, so the transform is looked up once per frame and cached.
    std::optional<Eigen::Isometry3d> lookupImuTransform(const std::string& imu_frame) {
        if (imu_to_base_ && imu_frame == imu_frame_) {
            return imu_to_base_;
        }
        try {
            const auto tf =
                tf_buffer_->lookupTransform(base_frame_, imu_frame, tf2::TimePointZero);
            const auto& q = tf.transform.rotation;
            const auto& t = tf.transform.translation;
            Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
            iso.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
            iso.translation() = Eigen::Vector3d(t.x, t.y, t.z);
            imu_to_base_ = iso;
            imu_frame_ = imu_frame;
            RCLCPP_INFO(get_logger(), "IMU transform %s -> %s acquired", imu_frame.c_str(),
                        base_frame_.c_str());
            return imu_to_base_;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "no transform %s -> %s (%s) — commanding zero torque",
                                 imu_frame.c_str(), base_frame_.c_str(), ex.what());
            return std::nullopt;
        }
    }

    void publishZeroTorque() {
        ros2can_msgs::msg::DvControlTargetTv out;  // torque targets default to zero
        out.dv_steering_angle_target_tv =
            steering_cmd_.steering_angle_ * (180.0 / std::numbers::pi);
        torque_pub_->publish(out);
    }

    void publishDebug(const PipelineResult& result) {
        const auto to_msg = [](const std::array<double, 4>& src) {
            std_msgs::msg::Float64MultiArray msg;
            msg.data.assign(src.begin(), src.end());
            return msg;
        };

        tire_fx_pub_->publish(to_msg(result.forces.fx_));
        tire_fy_pub_->publish(to_msg(result.forces.fy_));
        tire_fz_pub_->publish(to_msg(result.forces.fz_));

        // [mz (Nm), fx_total (N)]
        std_msgs::msg::Float64MultiArray yaw_msg;
        yaw_msg.data = {result.yaw_cmd.mz_, result.yaw_cmd.fx_total_};
        yaw_moment_pub_->publish(yaw_msg);

        // [FL, FR, RL, RR] (Nm)
        std_msgs::msg::Float64MultiArray torque_msg;
        torque_msg.data = {result.torques[0], result.torques[1], result.torques[2],
                           result.torques[3]};
        wheel_torques_pub_->publish(torque_msg);

        // Three blocks of [forces, yaw, alloc, total] in µs: this cycle, mean, stddev.
        std_msgs::msg::Float64MultiArray timing_msg;
        timing_msg.data = {
            result.timings.forces_us,    result.timings.yaw_us,        result.timings.alloc_us,
            result.timings.total_us,     result.timings_avg.forces_us, result.timings_avg.yaw_us,
            result.timings_avg.alloc_us, result.timings_avg.total_us,  result.timings_std.forces_us,
            result.timings_std.yaw_us,   result.timings_std.alloc_us,  result.timings_std.total_us};
        timing_pub_->publish(timing_msg);

        // [r_ref, r, r_ref − r, vx_ref, vx, vx_ref − vx]
        std_msgs::msg::Float64MultiArray tracking_msg;
        tracking_msg.data = {
            steering_cmd_.r_ref_,  state_.yaw_rate_, steering_cmd_.r_ref_ - state_.yaw_rate_,
            steering_cmd_.vx_ref_, state_.vx_,       steering_cmd_.vx_ref_ - state_.vx_};
        tracking_pub_->publish(tracking_msg);

#ifdef DEBUG
        // Achieved [Mz, Fx_total]: undo the allocator's output map
        // (F = T·gear_ratio / R), then apply G.
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
        // CarCommand.steering is in degrees; the pipeline works in radians throughout.
        const double steering_rad = msg->steering * (std::numbers::pi / 180.0);

        steering_cmd_.steering_angle_ = steering_rad;
        steering_cmd_.vx_ref_ = msg->velocity;
        steering_cmd_.r_ref_ = msg->yaw_rate;
        state_.steering_angle_ = steering_rad;  // mirror into VehicleState

        last_cmd_rx_ = now();
        have_cmd_ = true;
    }

    // Body-frame velocities from odometry; consumed on the next IMU tick.
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr& msg) {
      if (first_odom_recieved_) {
          // 200 ms odometry filter, assumes odometry at 100 hz
          state_.vx_ -= state_.vx_ / 20;
          state_.vy_ -= state_.vy_ / 20;
          state_.vx_ += msg->twist.twist.linear.x / 20;
          state_.vy_ += msg->twist.twist.linear.y / 20;
      } else {
          first_odom_recieved_ = true;
          state_.vx_ = msg->twist.twist.linear.x;
          state_.vy_ = msg->twist.twist.linear.y;
      }
      last_odom_rx_ = now();
      have_odom_ = true;
    }

    // Per-wheel ω [FL, FR, RL, RR] (rad/s) for the DFO, converted from motor rpm. The CAN
    // message carries no header, so the sample period comes from consecutive receive times.
    void wheelSpeedsCallback(const ros2can_msgs::msg::VehicleStatusWheelSpeed::SharedPtr& msg) {
        constexpr double motor_rpm_to_wheel_rad_s =
            2.0 * std::numbers::pi / 60.0 / VehicleConfig::gear_ratio;
        WheelOmegas omegas;
        omegas << msg->wheel_speed_fl, msg->wheel_speed_fr, msg->wheel_speed_rl,
            msg->wheel_speed_rr;
        omegas *= motor_rpm_to_wheel_rad_s;

        const rclcpp::Time stamp = now();
        double dt = 0.0;
        if (have_last_wheel_stamp_ && stamp > last_wheel_stamp_) {
            dt = (stamp - last_wheel_stamp_).seconds();
        }
        last_wheel_stamp_ = stamp;
        have_last_wheel_stamp_ = true;

        orchestrator_.updateWheelState(motor_torques_, omegas, dt);
    }

    // Cached for the DFO — published at a slower rate than the wheel speeds.
    void motorTorquesCallback(const ros2can_msgs::msg::VehicleStatusMotorTorques::SharedPtr& msg) {
        motor_torques_ << msg->torque_fl, msg->torque_fr, msg->torque_rl, msg->torque_rr;
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ros2can_msgs::msg::VehicleStatusWheelSpeed>::SharedPtr
        wheel_speeds_sub_;
    rclcpp::Subscription<ros2can_msgs::msg::VehicleStatusMotorTorques>::SharedPtr
        motor_torques_sub_;
    rclcpp::Subscription<fs_msgs::msg::CarCommand>::SharedPtr steering_sub_;
    rclcpp::Publisher<ros2can_msgs::msg::DvControlTargetTv>::SharedPtr torque_pub_;

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
    VehicleState state_{};
    SteeringCommand steering_cmd_{};

    // Latest per-wheel motor-torque feedback [FL, FR, RL, RR] (Nm) for the DFO.
    WheelTorques motor_torques_{WheelTorques::Zero()};

    // Previous IMU stamp — derives the control period.
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
    bool have_last_stamp_{false};

    // Previous wheel-speed receive time (node clock) — derives the DFO sample period.
    rclcpp::Time last_wheel_stamp_{0, 0, RCL_ROS_TIME};
    bool have_last_wheel_stamp_{false};

    // Receive times for the input watchdog (node clock).
    rclcpp::Time last_cmd_rx_{0, 0, RCL_ROS_TIME};
    bool have_cmd_{false};
    rclcpp::Time last_odom_rx_{0, 0, RCL_ROS_TIME};
    bool have_odom_{false};

    bool first_imu_recieved_{false};
    bool first_odom_recieved_{false};

    std::size_t alloc_failures_{0};

    // IMU → base_frame transform, cached once the static transform is available.
    std::string base_frame_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string imu_frame_;
    std::optional<Eigen::Isometry3d> imu_to_base_;

    // Previous base-frame ω — differentiated for the lever-arm angular acceleration.
    Eigen::Vector3d last_omega_{Eigen::Vector3d::Zero()};
    bool have_last_omega_{false};
};

template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
TVNode(Orchestrator<F, Y, A>) -> TVNode<F, Y, A>;

template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
TVNode(Orchestrator<F, Y, A>, const std::string&) -> TVNode<F, Y, A>;

}  // namespace tv
