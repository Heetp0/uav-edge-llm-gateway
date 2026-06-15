#!/usr/bin/env python3
"""
benchmark.launch.py
-------------------
Launches the safety filter and benchmark pipeline together.
This is the correct way to run the full benchmark — the safety
filter must be up before the benchmark starts publishing to
/llm/raw_output, otherwise early commands are dropped.

Usage:
    ros2 launch drone_safety benchmark.launch.py
    ros2 launch drone_safety benchmark.launch.py model:=qwen2.5:7b
    ros2 launch drone_safety benchmark.launch.py log_level:=debug

After the benchmark completes, run the plotter:
    ros2 run drone_safety plot_results
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="qwen2.5:3b",
        description="Ollama model name to benchmark (e.g. qwen2.5:3b, qwen2.5:7b)",
    )

    # Safety filter must be up first — it subscribes to /llm/raw_output
    # and publishes /fmu/in/trajectory_setpoint continuously at 20 Hz.
    safety_filter_node = Node(
        package="drone_safety",
        executable="safety_filter_node",
        name="safety_filter_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[config],
    )

    # Benchmark node starts 2 seconds after the safety filter to ensure
    # the subscriber is ready before the first LLM output is published.
    benchmark_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="drone_safety",
                executable="benchmark_pipeline",
                name="llm_benchmark_node",
                output="screen",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                parameters=[config],
            )
        ],
    )

    return LaunchDescription([
        log_level_arg,
        model_arg,
        safety_filter_node,
        benchmark_node,
    ])
