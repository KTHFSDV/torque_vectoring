#!/usr/bin/env python3
"""
Launch file for torque_vectoring_pkg.

Starts the torque-vectoring node with its topics remapped onto the
vehicle bus / navigation stack.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for torque_vectoring_pkg."""

    return LaunchDescription(
        [
            Node(
                package="torque_vectoring_pkg",
                executable="torque_vectoring_node",
                name="torque_vectoring_node",
                output="screen",
                parameters=[{"base_frame": "base_link"}],
                remappings=[
                    ("cmd_in", "navigation/dv_control_target"),
                    ("odom", "/slam/output/odom"),
                    ("imu", "/ouster/imu"),
                    ("wheel_speeds", "/ros2can/recv/vehicle_status_wheel_speed"),
                    ("motor_torques", "/ros2can/recv/vehicle_status_motor_torques"),
                    ("cmd_out", "/navigation/dv_control_target_tv"),
                ],
            ),
        ]
    )
