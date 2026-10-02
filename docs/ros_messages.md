# ROS2 Messages

Topic names are as remapped in [launch.py](../launch/launch.py). Wheel order in all arrays is `[FL, FR, RL, RR]`.

## Input

### `/navigation/dv_control_target`

- **Type**: `fs_msgs/msg/CarCommand`
- **Purpose**: Steering angle (deg), target velocity and target yaw rate from the control stack.

### `/odom`

- **Type**: `nav_msgs/msg/Odometry`
- **Purpose**: Longitudinal and lateral velocity. SensorData QoS.

### `/imu`

- **Type**: `sensor_msgs/msg/Imu`
- **Purpose**: Yaw rate and accelerations. Each IMU message triggers one control cycle, so the output rate follows the IMU rate. SensorData QoS.

### `/vehicle/wheel_speeds`

- **Type**: `fs_msgs/msg/Wheelspeeds`
- **Purpose**: Per-wheel angular speeds. SensorData QoS.

### `/ros2can/receive/vehicle_status_motor_torques`

- **Type**: `ros2can_msgs/msg/VehicleStatusMotorTorques`
- **Purpose**: Measured motor torques.

## Output

### `/ros2can/send/dv_control_target_tv`

- **Type**: `ros2can_msgs/msg/DvControlTargetTv`
- **Purpose**: Per-wheel torque targets and steering angle target (deg) sent to the car over CAN. Zero torque is commanded if `cmd_in` or `odom` goes stale.

### Debug topics

All `std_msgs/msg/Float64MultiArray` unless noted.

| Topic | Content |
| --- | --- |
| `/debug/tire_fx`, `/debug/tire_fy`, `/debug/tire_fz` | Estimated tire forces per wheel |
| `/debug/yaw_moment` | `[Mz, Fx_total]` demand from the yaw controller |
| `/debug/wheel_torques` | Allocated wheel torques |
| `/debug/timing` | Pipeline timing |
| `/debug/tracking` | Yaw rate tracking |
| `/debug/alloc_yaw_moment`, `/debug/alloc_fx_total` | `std_msgs/msg/Float64`, achieved demand after allocation (only built with `-DDEBUG`) |
