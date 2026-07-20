#pragma once
#include <vehicle_params_cpp/dev19.hpp>

// =============================================================================
// config.hpp — edit all tunable parameters here.
//
// Each struct maps to one pipeline component. Pass the values to the
// component constructors as shown in the usage example at the bottom.
// =============================================================================

namespace tv {

// Standard gravity [m/s²] — shared by the load-transfer model and grip clamps.
inline constexpr double kGravity{9.81};

// -----------------------------------------------------------------------------
// Vehicle physical properties — shared across all three layers
// -----------------------------------------------------------------------------
struct VehicleConfig {
    static constexpr double mass{kthfs::vehicle::dev19::weight_kg};  // total vehicle mass [kg]
    static constexpr double cg_height{
        kthfs::vehicle::dev19::cg_height_m};  // centre-of-gravity height [m]
    static constexpr double wheelbase{
        kthfs::vehicle::dev19::wheelbase_m};  // front-to-rear axle distance [m]
    static constexpr double l_r{0.705};       // rear axle to CG [m]  →  l_f = wheelbase - l_r
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

// -----------------------------------------------------------------------------
// Aerodynamic and rolling-resistance losses — reserved for a future Fx loss
// feedforward in the longitudinal tracker (currently unused)
// -----------------------------------------------------------------------------
struct AeroConfig {
    static constexpr double rho{1.225};  // air density [kg/m³]  (1.225 at sea level, 15 °C)
    static constexpr double c_d{kthfs::vehicle::dev19::drag_coefficient};  // drag coefficient
    static constexpr double frontal_area{
        kthfs::vehicle::dev19::frontal_area};  // frontal reference area [m²]
    static constexpr double c_rr{
        kthfs::vehicle::dev19::rolling_resistance};  // rolling resistance coefficient
};

// -----------------------------------------------------------------------------
// Driving Force Observer — KinematicForceEstimator
// -----------------------------------------------------------------------------
struct ForceEstimatorConfig {
    static constexpr double j_w{0.5};       // rotational inertia referred to the wheel [kg·m²]
                                            // (includes motor inertia reflected as J_motor·N²)
    static constexpr double eta{1.0};       // drivetrain efficiency (motor → wheel), (0, 1]
    static constexpr double lpf_tau{0.10};  // DFO low-pass filter time constant [s]
                                            // smaller → faster but noisier Fx estimate
};

// -----------------------------------------------------------------------------
// Bicycle model — reserved for a future steady-state yaw-moment feedforward
// (currently unused)
// -----------------------------------------------------------------------------
struct BicycleModelConfig {
    // dev19::cornerings_stiffness_n_rad is a per-TIRE value; the bicycle model needs
    // per-AXLE stiffness, so multiply by 2 (two tires per axle).
    static constexpr double c_f{kthfs::vehicle::dev19::cornerings_stiffness_n_rad *
                                2.0};  // front axle cornering stiffness [N/rad]
    static constexpr double c_r{kthfs::vehicle::dev19::cornerings_stiffness_n_rad *
                                2.0};  // rear axle cornering stiffness [N/rad]
                                       // typical FS values: 40 000–70 000 N/rad
};

struct PIDYawControllerConfig {
    static constexpr double kp{250};  // Proportional term for yaw rate error tracking
    static constexpr double ki{50};   // Integral term for yaw rate error tracking
    static constexpr double kd{20};   // Derivative term on the yaw-rate error
    static constexpr double i_max{
        1000.0};  // yaw integral-term saturation [Nm] — symmetric anti-windup clamp.
                  // Tune to the yaw-moment authority the allocator can realise.
    static constexpr double d_tau{
        0.02};  // low-pass time constant on the derivative term [s]. The yaw-rate
                // error comes straight from the gyro, so the raw derivative is
                // noise-dominated; this filters it before applying kd.
};

struct LongitudinalPiConfig {
    static constexpr double k_p{2.0};  // proportional gain [1/s] — velocity-loop bandwidth.
                                       // Full Fx (grip clamp μg≈15.7 m/s²) is reached at a
                                       // velocity error of μg/k_p; keep k_p low (~1–3) so
                                       // tracking noise doesn't slam the demand to saturation.
    static constexpr double k_i{1.0};  // integral gain     [1/s²]
    static constexpr double b{0.8};    // setpoint weight on the proportional term (2-DoF PI),
                                       // [0, 1]. 1 → classic PI; <1 softens the proportional
                                       // kick on reference steps without changing disturbance
                                       // rejection (the integral still sees the full error).
    static constexpr double eta{1.0};  // drivetrain efficiency
    static constexpr double vx_ref_rate{
        5.0};  // max rate of change of the velocity reference [m/s²]. Ramps vx_ref so a
               // step target (e.g. launch from rest) doesn't slam Fx to saturation.
               // Keep below the grip limit μ·g ≈ 15.7 m/s².
};

struct AllocatorConfig {
    static constexpr double w_fx{4.0};    // Fx tracking weight
    static constexpr double w_mz{10.0};   // Mz tracking weight
    static constexpr double w_reg{0.01};  // torque magnitude regularisation
};

// -----------------------------------------------------------------------------
// ROS node behaviour
// -----------------------------------------------------------------------------
struct NodeConfig {
    static constexpr double input_timeout{
        0.3};  // max age of the latest cmd_in / odom message [s]. If either input
               // is older (or was never received), the node commands zero torque
               // instead of acting on stale references.
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
