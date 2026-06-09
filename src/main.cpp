
#include <memory>

#include "config.hpp"
#include "orchestrator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_interface.hpp"
#include "tv/kinematic_force_estimator.hpp"
#include "tv/pid_yaw_controller.hpp"
#include "tv/qp_torque_allocator.hpp"
#include "types.hpp"

namespace tv {

int main(int argc, char* argv[]) {
    KinematicForceEstimator kfe(tv::Config::force_estimator);
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_unique<TVNode<KinematicForceEstimator, PIDYawController, QpTorqueAllocator>>());
    rclcpp::shutdown();
    return 0;
}
}  // namespace tv
