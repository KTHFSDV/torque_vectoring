#pragma once

#include <fs_msgs/msg/car_command.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "fs_msgs/msg/car_command.hpp"
#include "orchestrator.hpp"
#include "ros2can_msgs/msg/dv_control_target_tv.hpp"
#include "types.hpp"

namespace tv {

/// ROS2 node that wraps the full torque-vectoring pipeline.
///
/// Subscriptions:
///   ~/imu/data              sensor_msgs/Imu               — yaw_rate, ax, ay
///   ~/steering_command      std_msgs/Float64MultiArray[3]  — [steering_angle (rad), vx_ref (m/s),
///   r_ref (rad/s)]
///
/// Publication:
///   ~/wheel_torques         std_msgs/Float64MultiArray[4]  — [T_FL, T_FR, T_RL, T_RR] (Nm)
///
/// NOTE: VehicleState fields vx and vy (indices 0 and 1) are initialised to zero and
///       never updated by this node. Add an odometry subscriber if the controllers
///       require actual body velocities.
///
/// The pipeline is triggered on every IMU message. Steering updates are applied
/// immediately on receipt and will take effect on the next IMU tick.
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
class TVNode : public rclcpp::Node {
   public:
    explicit TVNode(Orchestrator<F, Y, A> orchestrator,
                    const std::string& node_name = "torque_vectoring")
        : Node(node_name), orchestrator_(std::move(orchestrator)) {
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::SharedPtr msg) { imuCallback(msg); });

        steering_sub_ = create_subscription<fs_msgs::msg::CarCommand>(
            "/navigation/dv_control_target_tv", 10,
            [this](fs_msgs::msg::CarCommand::SharedPtr msg) { steeringCallback(msg); });

        torque_pub_ = create_publisher<ros2can_msgs::msg::DvControlTargetTv>(
            "ros2can/send/dv_control_target_tv", 10);
    }

   private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr& msg) {
        state_.yaw_rate_ = msg->angular_velocity.z;  // yaw_rate  [rad/s]
        state_.ax_ = msg->linear_acceleration.x;     // ax        [m/s²]
        state_.ay_ = msg->linear_acceleration.y;     // ay        [m/s²]

        const WheelTorques torques = orchestrator_.run(state_, steering_cmd_);

        ros2can_msgs::msg::DvControlTargetTv out;
        out.dv_steering_angle_target_tv = steering_cmd_.steering_angle_;
        out.dv_fl_speedd_target = torques[0];
        out.dv_fr_speed_target = torques[1];
        out.dv_rl_speed_target = torques[2];
        out.dv_rr_speed_target = torques[3];
        torque_pub_->publish(out);
    }

    void steeringCallback(const fs_msgs::msg::CarCommand::SharedPtr& msg) {
        steering_cmd_.steering_angle_ = msg->steering;  // steering_angle [rad]
        steering_cmd_.vx_ref_ = msg->velocity;          // vx_ref         [m/s]
        steering_cmd_.r_ref_ = msg->yaw_rate;           // r_ref          [rad/s]
        state_.steering_angle_ = msg->steering;         // mirror into VehicleState
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<fs_msgs::msg::CarCommand>::SharedPtr steering_sub_;
    rclcpp::Publisher<ros2can_msgs::msg::DvControlTargetTv>::SharedPtr torque_pub_;

    Orchestrator<F, Y, A> orchestrator_;
    VehicleState state_{};            // vx_, vy_, yaw_rate_, steering_angle_, ax_, ay_
    SteeringCommand steering_cmd_{};  // steering_angle_, vx_ref_, r_ref_
};

// CTAD deduction guides
template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
TVNode(Orchestrator<F, Y, A>) -> TVNode<F, Y, A>;

template <ForceEstimator F, YawMomentGenerator Y, TorqueAllocator A>
TVNode(Orchestrator<F, Y, A>, const std::string&) -> TVNode<F, Y, A>;

}  // namespace tv
