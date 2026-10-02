#pragma once
#include <vehicle_params_cpp/dev19.hpp>

namespace tv {

inline constexpr double kGravity{9.81};  // [m/s²]

struct VehicleConfig {
    static constexpr double mass{kthfs::vehicle::dev19::weight_kg};  // total vehicle mass [kg]
    static constexpr double cg_height{
        kthfs::vehicle::dev19::cg_height_m};  // centre-of-gravity height [m]
    static constexpr double wheelbase{
        kthfs::vehicle::dev19::wheelbase_m};  // front-to-rear axle distance [m]
    static constexpr double l_r{0.612};       // rear axle to CG [m]  →  l_f = wheelbase - l_r
    static constexpr double trackwidth_front{
        kthfs::vehicle::dev19::track_width_m};  // front track width [m]
    static constexpr double trackwidth_rear{
        kthfs::vehicle::dev19::track_width_m};  // rear track width [m]
    static constexpr double front_tlltd{
        0.40};  // front TLLTD fraction (0 = all rear, 1 = all front)
    static constexpr double wheel_radius{
        kthfs::vehicle::dev19::tire_radius_m};  // effective wheel radius [m]
    static constexpr double mu{1.6};            // tyre-road friction coefficient
    static constexpr double gear_ratio{13.5};   // motor → wheel gear ratio
    static constexpr double motor_t_max{21.0};  // per-wheel motor peak torque [Nm]
    static constexpr double t_max{motor_t_max *
                                  gear_ratio};  // per-wheel wheel-side torque cap [Nm]
};

// Reserved for a future Fx loss feedforward in the longitudinal tracker (unused).
struct AeroConfig {
    static constexpr double rho{1.225};  // air density [kg/m³]  (1.225 at sea level, 15 °C)
    static constexpr double c_d{kthfs::vehicle::dev19::drag_coefficient};  // drag coefficient
    static constexpr double frontal_area{
        kthfs::vehicle::dev19::frontal_area};  // frontal reference area [m²]
    static constexpr double c_rr{
        kthfs::vehicle::dev19::rolling_resistance};  // rolling resistance coefficient
};

struct ForceEstimatorConfig {
    static constexpr double j_w{0.5};       // inertia referred to the wheel [kg·m²], incl. J_m·N²
    static constexpr double eta{1.0};       // drivetrain efficiency (motor → wheel), (0, 1]
    static constexpr double lpf_tau{0.10};  // DFO LPF time constant [s]; smaller → noisier
};

// Reserved for a future steady-state yaw-moment feedforward (unused).
struct BicycleModelConfig {
    // dev19::cornerings_stiffness_n_rad is per-TIRE; the bicycle model needs per-AXLE.
    static constexpr double c_f{kthfs::vehicle::dev19::cornerings_stiffness_n_rad * 2.0};
    static constexpr double c_r{kthfs::vehicle::dev19::cornerings_stiffness_n_rad * 2.0};
};

struct PIDYawControllerConfig {
    static constexpr double kp{600};
    static constexpr double ki{1};
    static constexpr double kd{0};
    static constexpr double i_max{250.0};  // yaw integral-term clamp [Nm] — size to the yaw
                                           // authority the allocator can realise
    static constexpr double d_tau{0.02};   // derivative low-pass time constant [s]
};

struct LongitudinalPiConfig {
    static constexpr double k_p{6.0};           // [1/s] — full Fx (grip clamp μg) is reached at a
                                                // velocity error of μg/k_p; keep low so tracking
                                                // noise doesn't slam the demand to saturation
    static constexpr double k_i{0.5};           // [1/s²]
    static constexpr double b{0.8};             // setpoint weight on the proportional term, [0, 1]
    static constexpr double eta{1.0};           // drivetrain efficiency
    static constexpr double vx_ref_rate{20.0};  // velocity-reference slew limit [m/s²]
};

struct AllocatorConfig {
    static constexpr double w_fx{4.0};    // Fx tracking weight
    static constexpr double w_mz{10.0};   // Mz tracking weight
    static constexpr double w_reg{0.01};  // torque magnitude regularisation
};

struct NodeConfig {
    static constexpr double input_timeout{0.3};  // max age of cmd_in / odom [s]; stale or
                                                 // missing inputs command zero torque
};

struct Config {
    static constexpr VehicleConfig vehicle{};
    static constexpr AeroConfig aero{};
    static constexpr ForceEstimatorConfig force_estimator{};
    static constexpr BicycleModelConfig bicycle{};
    static constexpr PIDYawControllerConfig pid_yaw{};
    static constexpr LongitudinalPiConfig longitudinal_pi{};
    static constexpr AllocatorConfig allocator{};
};

}  // namespace tv
