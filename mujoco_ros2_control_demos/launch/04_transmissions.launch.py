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
Tutorial 4: Transmissions

This tutorial demonstrates using ros2_control transmissions with MuJoCo.
Transmissions allow mapping between actuators and joints with mechanical
reduction ratios and offsets.

Key concepts:
- SimpleTransmission interface from transmission_interface
- Mechanical reduction configuration (gear ratios)
- Actuator-to-joint mapping in MJCF
- Runtime MJCF modification to rename joints to actuators

Resources used:
- demo_resources/scenes/scene.xml (base scene)
- demo_resources/robot/test_robot.xml (modified at runtime for actuator names)
- test_robot.urdf with use_transmissions:=true

Usage:
    ros2 launch mujoco_ros2_control_demos 04_transmissions.launch.py
    ros2 launch mujoco_ros2_control_demos 04_transmissions.launch.py headless:=true
"""

import os
import tempfile

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


def process_transmission_files(robot_description_str, mujoco_model_path):
    """
    Convert joints to actuators in the robot description for using transmission interfaces.
    The MJCF model is modified to rename joint1/joint2 to actuator1/actuator2 to match
    the transmission configuration in the URDF.
    """
    with open(mujoco_model_path) as f:
        scene_content = f.read()

    if '<include file="' in scene_content:
        include_file = scene_content.split('<include file="')[1].split('"/>')[0].strip()
        include_path = os.path.join(os.path.dirname(mujoco_model_path), include_file)
        with open(include_path) as f:
            include_content = f.read()

        # Replace joint1 and joint2 with actuator1 and actuator2
        include_content = include_content.replace('"joint1"', '"actuator1"')
        include_content = include_content.replace('"joint2"', '"actuator2"')

        # Copy to temp
        temp_include = tempfile.NamedTemporaryFile(delete=False, suffix=".xml", mode="w")
        temp_include.write(include_content)
        temp_include.close()

        # Replace include file path in scene_content
        scene_content = scene_content.replace(include_file, temp_include.name)

    # Write to a temporary file
    temp_scene_file = tempfile.NamedTemporaryFile(delete=False, suffix=".xml", mode="w")
    temp_scene_file.write(scene_content)
    temp_scene_file.close()

    robot_description_str = robot_description_str.replace(mujoco_model_path, temp_scene_file.name)

    print("Modified scene file with transmissions at:", temp_scene_file.name)
    return robot_description_str


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("mujoco_ros2_control_demos")

    use_pid = LaunchConfiguration("use_pid").perform(context) == "true"

    # Build robot description with transmissions enabled
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "demo_resources", "robot", "test_robot.urdf"]),
            " use_pid:=",
            "true" if use_pid else "false",
            " headless:=",
            LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=false",
            " use_transmissions:=true",
        ]
    )

    robot_description_str = robot_description_content.perform(context)

    # Process transmissions - modify MJCF to use actuator names
    if "mujoco_model" in robot_description_str:
        mujoco_model_path = robot_description_str.split('mujoco_model">')[1].split("</param>")[0].strip()
        robot_description_str = process_transmission_files(robot_description_str, mujoco_model_path)

    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    parameters_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

    nodes = []

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

    use_pid = DeclareLaunchArgument(
        "use_pid",
        default_value="false",
        description="If true, enables PID control for velocity/effort modes",
    )

    return LaunchDescription(
        [
            headless,
            use_pid,
            OpaqueFunction(function=launch_setup),
        ]
    )
