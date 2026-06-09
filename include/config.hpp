#pragma once
#include <vehicle_params_cpp/dev19.hpp>

// =============================================================================
// config.hpp — edit all tunable parameters here.
//
// Each struct maps to one pipeline component. Pass the values to the
// component constructors as shown in the usage example at the bottom.
// =============================================================================

namespace tv {

// -----------------------------------------------------------------------------
// Vehicle physical properties — shared across all three layers
// -----------------------------------------------------------------------------
struct VehicleConfig {
    static constexpr double mass{kthfs::vehicle::dev19::weight_kg};  // total vehicle mass [kg]
    static constexpr double cg_height{
        kthfs::vehicle::dev19::cg_height_m};  // centre-of-gravity height [m]
    static constexpr double wheelbase{
        kthfs::vehicle::dev19::wheelbase_m};  // front-to-rear axle distance [m]
    static constexpr double l_r{0.82};        // rear axle to CG [m]  →  l_f = wheelbase - l_r
    static constexpr double trackwidth_front{
        kthfs::vehicle::dev19::track_width_m};  // front track width [m]
    static constexpr double trackwidth_rear{
        kthfs::vehicle::dev19::track_width_m};  // rear track width [m]
    static constexpr double front_tlltd{
        0.45};  // front TLLTD fraction (0 = all rear, 1 = all front)
    static constexpr double wheel_radius{
        kthfs::vehicle::dev19::tire_radius_m};  // effective wheel radius [m]
    static constexpr double mu{1.6};            // tyre-road friction coefficient
    static constexpr double t_max{21.0 * 14};   // per-wheel motor torque limit [Nm]
};

// -----------------------------------------------------------------------------
// Aerodynamic and rolling-resistance losses — SmcYawController Fx feedforward
// -----------------------------------------------------------------------------
struct AeroConfig {
    static constexpr double rho{1.225};  // air density [kg/m³]  (1.225 at sea level, 15 °C)
    static constexpr double c_d{kthfs::vehicle::dev19::drag_coefficient};  // drag coefficient
    static constexpr double frontal_area{
        kthfs::vehicle::dev19::frontal_area};  // frontal reference area [m²]
    static constexpr double c_rr =
        0.0 {kthfs::vehicle::dev19::rolling_resistance};  // rolling resistance coefficient
};

// -----------------------------------------------------------------------------
// Driving Force Observer — KinematicForceEstimator
// -----------------------------------------------------------------------------
struct ForceEstimatorConfig {
    static constexpr double j_w{0.5};       // wheel + motor rotational inertia [kg·m²]
    static constexpr double lpf_tau{0.02};  // DFO low-pass filter time constant [s]
                                            // smaller → faster but noisier Fx estimate
};

// -----------------------------------------------------------------------------
// Bicycle model — SmcYawController feedforward (steady-state yaw moment)
// -----------------------------------------------------------------------------
struct BicycleModelConfig {
    static constexpr double c_f{
        kthfs::vehicle::dev19::cornerings_stiffness_n_rad};  // front axle cornering stiffness
                                                             // [N/rad]
    static constexpr double c_r{
        kthfs::vehicle::dev19::cornerings_stiffness_n_rad};  // rear axle cornering stiffness
                                                             // [N/rad] typical FS values: 40 000–70
                                                             // 000 N/rad
};

struct PIDYawControllerConfig {
    static constexpr double kp{0.5};  // Proportional term for yaw rate error tracking
    static constexpr double ki{0.1};  // Integral term for yaw rate error tracking
};

struct LongitudinalPiConfig {
    static constexpr double k_p{2.0};  // proportional gain [1/s]
    static constexpr double k_i{0.0};  // integral gain     [1/s²]
    static constexpr double eta{0.0};  // drivetrain efficiency
};

struct AllocatorConfig {
    static constexpr double w_fx{1.0};     // Fx tracking weight
    static constexpr double w_mz{10.0};    // Mz tracking weight
    static constexpr double w_reg{0.001};  // torque magnitude regularisation
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
