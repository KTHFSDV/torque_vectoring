#pragma once

#include <array>

#include <Eigen/Dense>

namespace tv {

// Populated from the IMU, odometry, and the steering sensor.
struct VehicleState {
    double vx_{0.0};             // longitudinal velocity [m/s]
    double vy_{0.0};             // lateral velocity      [m/s]
    double yaw_rate_{0.0};       // yaw rate              [rad/s]
    double steering_angle_{0.0}; // front wheel steer     [rad]
    double ax_{0.0};             // longitudinal accel    [m/s²]
    double ay_{0.0};             // lateral accel         [m/s²]
};

// Per-wheel tire forces, wheel order: FL, FR, RL, RR.
struct ForceVector {
    std::array<double, 4> fx_{};  // longitudinal forces [N]
    std::array<double, 4> fy_{};  // lateral forces      [N]
    std::array<double, 4> fz_{};  // normal forces       [N]
};

struct YawMomentCommand {
    double mz_{0.0};        // corrective yaw moment [Nm]
    double fx_total_{0.0};  // total longitudinal force demand [N]
};

// Driver/planner inputs.
struct SteeringCommand {
    double steering_angle_{0.0};  // front wheel steer angle [rad]
    double vx_ref_{0.0};          // longitudinal velocity reference [m/s]
    double r_ref_{0.0};           // yaw rate reference [rad/s]
};

// Per-wheel torque commands [FL, FR, RL, RR] (Nm).
using WheelTorques = Eigen::Vector4d;

// Per-wheel angular velocities [FL, FR, RL, RR] (rad/s).
using WheelOmegas = Eigen::Vector4d;

// Per-stage pipeline execution times [µs] — telemetry only, no control meaning.
struct StageTimings {
    double forces_us{0.0};  // force estimator (stage 1)
    double yaw_us{0.0};     // yaw moment generator (stage 2)
    double alloc_us{0.0};   // torque allocator (stage 3)
    double total_us{0.0};   // full pipeline
};

// Full pipeline output. The intermediates exist so the ROS node can publish them
// on debug topics.
struct PipelineResult {
    ForceVector forces{};        // tyre force estimate (stage 1)
    YawMomentCommand yaw_cmd{};  // yaw moment + Fx demand (stage 2)
    WheelTorques torques{WheelTorques::Zero()};  // allocated wheel torques (stage 3)
    StageTimings timings{};      // per-stage execution times, this cycle
    StageTimings timings_avg{};  // running mean since startup
    StageTimings timings_std{};  // running sample standard deviation since startup
    bool alloc_ok{true};  // false when the allocator failed and held its previous solution
};

}  // namespace tv
