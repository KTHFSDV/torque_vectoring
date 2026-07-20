
#include <memory>
#include <utility>

#include "config.hpp"
#include "orchestrator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_interface.hpp"
#include "tv/kinematic_force_estimator.hpp"
#include "tv/pid_yaw_controller.hpp"
#include "tv/ldlt_torque_allocator.hpp"
#include "types.hpp"

int main(int argc, char* argv[]) {
    using namespace tv;
    rclcpp::init(argc, argv);

    KinematicForceEstimator force_estimator;

    PIDYawController yaw_controller;  // all parameters from config.hpp

    LdltTorqueAllocator torque_allocator(VehicleConfig::wheel_radius, VehicleConfig::gear_ratio,
                                        VehicleConfig::trackwidth_front,
                                        VehicleConfig::trackwidth_rear, AllocatorConfig::w_fx,
                                        AllocatorConfig::w_mz, AllocatorConfig::w_reg);

    Orchestrator orchestrator(std::move(force_estimator), std::move(yaw_controller),
                              std::move(torque_allocator));

    rclcpp::spin(
        std::make_unique<TVNode<KinematicForceEstimator, PIDYawController, LdltTorqueAllocator>>(
            std::move(orchestrator)));
    rclcpp::shutdown();
    return 0;
}
