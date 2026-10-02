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
                remappings=[
                    ("cmd_in", "navigation/dv_control_target"),
                    ("odom", "/odom"),
                    ("imu", "/imu"),
                    ("wheel_speeds", "vehicle/wheel_speeds"),
                    ("motor_torques", "ros2can/receive/vehicle_status_motor_torques"),
                    ("cmd_out", "ros2can/send/dv_control_target_tv"),
                ],
            ),
        ]
    )
