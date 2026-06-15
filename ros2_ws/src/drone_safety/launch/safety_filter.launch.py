#!/usr/bin/env python3
"""
safety_filter.launch.py
-----------------------
Launches the C++ safety filter node in isolation.
Use this during SITL testing when you want to feed the filter
manually via `ros2 topic pub` without running the full benchmark.

Usage:
    ros2 launch drone_safety safety_filter.launch.py
    ros2 launch drone_safety safety_filter.launch.py log_level:=debug
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("drone_safety"),
        "config", "geofence_params.yaml"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 logging level: debug | info | warn | error | fatal",
    )

    safety_filter_node = Node(
        package="drone_safety",
        executable="safety_filter_node",
        name="safety_filter_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[config],
    )

    return LaunchDescription([
        log_level_arg,
        safety_filter_node,
    ])
