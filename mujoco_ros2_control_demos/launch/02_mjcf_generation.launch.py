#!/usr/bin/env python3
#
# Copyright 2026 PAL Robotics S.L.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Tutorial 2: MJCF Generation at Runtime

This tutorial demonstrates generating MJCF models from URDF at runtime.
Instead of pre-defining the MJCF file, the robot description is converted
dynamically using the robot_description_to_mjcf conversion script.

Key concepts:
- Runtime URDF to MJCF conversion using robot_description_to_mjcf.sh
- Using external input files (demo_resources/mjcf_generation/test_inputs.xml)
- Using <mujoco_inputs> tags embedded in URDF for customization
- Dynamic scene generation with cameras and sensors

Usage:
    # Using external input files for MJCF customization
    ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py

    # Using mujoco_inputs embedded directly in the URDF
    ros2 launch mujoco_ros2_control_demos 02_mjcf_generation.launch.py use_urdf_inputs:=true
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("mujoco_ros2_control_demos")

    # When use_urdf_inputs is true, use PID mode which has mujoco_inputs embedded in URDF
    use_urdf_inputs = LaunchConfiguration("use_urdf_inputs").perform(context) == "true"

    # Build robot description - MJCF from topic mode
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "demo_resources", "robot", "test_robot.urdf"]),
            " use_pid:=",
            "true" if use_urdf_inputs else "false",
            " headless:=",
            LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=true",
            " use_transmissions:=false",
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    parameters_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    nodes = []

    # MJCF conversion node - converts URDF to MJCF at runtime
    if use_urdf_inputs:
        # Use mujoco_inputs from URDF (embedded in the xacro when use_pid + use_mjcf_from_topic)
        nodes.append(
            Node(
                package="mujoco_ros2_control",
                executable="robot_description_to_mjcf.sh",
                output="both",
                emulate_tty=True,
                arguments=["--publish_topic", "/mujoco_robot_description"],
            )
        )
    else:
        # Use external input files from demo_resources/mjcf_generation/
        nodes.append(
            Node(
                package="mujoco_ros2_control",
                executable="robot_description_to_mjcf.sh",
                output="both",
                emulate_tty=True,
                arguments=[
                    "--robot_description",
                    robot_description_str,
                    "--m",
                    PathJoinSubstitution([pkg_share, "demo_resources", "mjcf_generation", "test_inputs.xml"]),
                    "--scene",
                    PathJoinSubstitution([pkg_share, "demo_resources", "scenes", "scene_info.xml"]),
                    "--publish_topic",
                    "/mujoco_robot_description",
                ],
            )
        )

    # Robot state publisher
    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    # ros2_control node with MuJoCo
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(parameters_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            on_exit=Shutdown(),
        )
    )

    # Controller spawners
    controllers_to_spawn = ["joint_state_broadcaster", "position_controller", "gripper_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", parameters_file],
                output="both",
            )
        )

    return nodes


def generate_launch_description():
    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )

    use_urdf_inputs = DeclareLaunchArgument(
        "use_urdf_inputs",
        default_value="false",
        description="When true, uses mujoco_inputs embedded in the URDF instead of external files",
    )

    return LaunchDescription(
        [
            headless,
            use_urdf_inputs,
            OpaqueFunction(function=launch_setup),
        ]
    )
