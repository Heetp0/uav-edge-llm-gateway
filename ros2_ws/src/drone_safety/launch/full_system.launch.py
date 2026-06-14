#!/usr/bin/env python3
"""
full_system.launch.py
---------------------
Launches the complete UAV LLM gateway stack:
  1. safety_filter_node  — C++ deterministic safety layer (immediate)
  2. llm_gateway_node    — Python LLM inference node (after 2s delay)

This is the runtime launch file for actual UAV operation (SITL or hardware).
The benchmark_pipeline is NOT included here — this is the production stack.

Prerequisites:
  - Ollama running:   ollama serve
  - Model pulled:     ollama pull qwen2.5:3b
  - PX4 SITL or hardware connected via XRCE-DDS bridge

Usage:
    ros2 launch drone_safety full_system.launch.py
    ros2 launch drone_safety full_system.launch.py model:=qwen2.5:7b
    ros2 launch drone_safety full_system.launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 logging level: debug | info | warn | error | fatal",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="qwen2.5:3b",
        description="Ollama model name to use for inference",
    )

    ollama_url_arg = DeclareLaunchArgument(
        "ollama_url",
        default_value="http://localhost:11434/api/generate",
        description="Ollama API endpoint URL",
    )

    # Safety filter starts immediately and holds BLIND_HALT state
    # until odometry arrives from PX4. It will NOT accept LLM commands
    # until home position is locked from the first valid odometry message.
    safety_filter_node = Node(
        package="drone_safety",
        executable="safety_filter_node",
        name="safety_filter_node",
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # LLM gateway starts after a 2 second delay to ensure the safety
    # filter subscriber is ready before the first command is published.
    # Replace 'llm_gateway_node' with the actual executable name once added.
    llm_gateway_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="drone_safety",
                executable="llm_gateway_node",
                name="llm_gateway_node",
                output="screen",
                arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
                parameters=[
                    {"model": LaunchConfiguration("model")},
                    {"ollama_url": LaunchConfiguration("ollama_url")},
                ],
            )
        ],
    )

    return LaunchDescription([
        log_level_arg,
        model_arg,
        ollama_url_arg,
        safety_filter_node,
        llm_gateway_node,
    ])
