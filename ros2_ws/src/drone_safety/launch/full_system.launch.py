#!/usr/bin/env python3
"""
full_system.launch.py
---------------------
Launches the complete UAV edge AI stack in one command:

  t=0s   Ollama server
  t=0s   PX4 SITL (Gazebo)
  t=0s   MicroXRCEAgent (DDS bridge)
  t=5s   safety_filter_node  (waits for DDS bridge to be ready)
  t=7s   llm_gateway_node    (waits for safety filter to be ready)

Usage:
    ros2 launch drone_safety full_system.launch.py
    ros2 launch drone_safety full_system.launch.py model:=qwen2.5:7b
    ros2 launch drone_safety full_system.launch.py log_level:=debug
    ros2 launch drone_safety full_system.launch.py headless:=false

Prerequisites:
    ollama pull qwen2.5:3b           (once, before first launch)
    px4_sitl path set via px4_dir    (default: ~/PX4-Autopilot)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────────

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="ROS 2 logging level: debug | info | warn | error | fatal",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="qwen2.5:3b",
        description="Ollama model name (must be pulled before launch)",
    )

    ollama_url_arg = DeclareLaunchArgument(
        "ollama_url",
        default_value="http://localhost:11434/api/generate",
        description="Ollama API endpoint URL",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run Gazebo headless (true) or with GUI (false)",
    )

    px4_dir_arg = DeclareLaunchArgument(
        "px4_dir",
        default_value=os.path.expanduser("~/PX4-Autopilot"),
        description="Absolute path to the PX4-Autopilot directory",
    )

    vehicle_arg = DeclareLaunchArgument(
        "vehicle",
        default_value="gz_x500_mono_cam",
        description="PX4 SITL vehicle target (e.g. gz_x500_mono_cam, gz_x500)",
    )

    # ── Config file ───────────────────────────────────────────────────────────

    config_file = os.path.join(
        get_package_share_directory("drone_safety"),
        "config",
        "geofence_params.yaml",
    )

    # ── Process 1: Ollama server ──────────────────────────────────────────────
    # Starts immediately. If already running, the process exits harmlessly.

    ollama_process = ExecuteProcess(
        cmd=["ollama", "serve"],
        name="ollama_server",
        output="screen",
        shell=False,
    )

    # ── Process 2: PX4 SITL + Gazebo ─────────────────────────────────────────
    # HEADLESS env var controls whether Gazebo GUI launches.

    px4_process = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "HEADLESS={headless} make -C {px4_dir} px4_sitl {vehicle}".format(
                headless="1",   # overridden at runtime via env
                px4_dir=os.path.expanduser("~/PX4-Autopilot"),
                vehicle="gz_x500_mono_cam",
            )
        ],
        name="px4_sitl",
        output="screen",
        additional_env={
            "HEADLESS": "1",
        },
        shell=False,
    )

    # ── Process 3: MicroXRCEAgent (DDS bridge) ────────────────────────────────
    # Bridges PX4 uORB topics to ROS 2 via XRCE-DDS over UDP.

    dds_bridge = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_dds_agent",
        output="screen",
        shell=False,
    )

    # ── Process 4: Safety filter node (t=5s) ─────────────────────────────────
    # Delayed 5s to allow DDS bridge and PX4 to establish the UDP connection.
    # Starts in BLIND_HALT state — safe to run before PX4 is fully booted.

    safety_filter_node = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[full_system] Starting safety_filter_node..."),
            Node(
                package="drone_safety",
                executable="safety_filter_node",
                name="safety_filter_node",
                output="screen",
                arguments=[
                    "--ros-args", "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[config_file],
            ),
        ],
    )

    # ── Process 5: LLM gateway node (t=7s) ───────────────────────────────────
    # Delayed 7s to ensure the safety filter subscriber is registered
    # before the first LLM output is published to /llm/raw_output.

    llm_gateway_node = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="[full_system] Starting llm_gateway_node..."),
            Node(
                package="drone_safety",
                executable="llm_gateway_node",
                name="llm_gateway_node",
                output="screen",
                arguments=[
                    "--ros-args", "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[
                    config_file,
                    {
                        "model":      LaunchConfiguration("model"),
                        "ollama_url": LaunchConfiguration("ollama_url"),
                    },
                ],
            ),
        ],
    )

    return LaunchDescription([
        # Arguments
        log_level_arg,
        model_arg,
        ollama_url_arg,
        headless_arg,
        px4_dir_arg,
        vehicle_arg,
        # Processes (t=0)
        ollama_process,
        px4_process,
        dds_bridge,
        # ROS 2 nodes (delayed)
        safety_filter_node,   # t=5s
        llm_gateway_node,     # t=7s
    ])
        ollama_url_arg,
        safety_filter_node,
        llm_gateway_node,
    ])
