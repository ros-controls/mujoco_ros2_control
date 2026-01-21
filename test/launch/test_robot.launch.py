#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

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
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, NotSubstitution
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def process_transmission_files(robot_description_str, mujoco_model_path):
    """
    If used, will convert joints to actuators in the robot description for using transmission
    interfaces. Contents will be writted to a modified temp file.
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
    print(robot_description_str)
    return robot_description_str


def launch_setup(context, *args, **kwargs):

    pkg_share = FindPackageShare("mujoco_ros2_control")

    # Build robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("mujoco_ros2_control"), "test_resources", "test_robot.urdf"]),
            " use_pid:=",
            LaunchConfiguration("use_pid"),
            " headless:=",
            LaunchConfiguration("headless"),
            " use_mjcf_from_topic:=",
            LaunchConfiguration("use_mjcf_from_topic"),
            " use_transmissions:=",
            LaunchConfiguration("test_transmissions"),
        ]
    )

    robot_description_str = robot_description_content.perform(context)

    # Process transmissions if needed
    if LaunchConfiguration("test_transmissions").perform(context) == "true":
        if "mujoco_model" in robot_description_str:
            mujoco_model_path = robot_description_str.split('mujoco_model">')[1].split("</param>")[0].strip()

            robot_description_str = process_transmission_files(robot_description_str, mujoco_model_path)

    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    nodes = []

    # Conditionally launch the conversion node depending on PID usage
    nodes.extend(
        [
            Node(
                package="mujoco_ros2_control",
                executable="robot_description_to_mjcf.sh",
                output="both",
                emulate_tty=True,
                arguments=[
                    "--robot_description",
                    robot_description_str,
                    "--m",
                    PathJoinSubstitution([pkg_share, "test_resources", "test_inputs.xml"]),
                    "--scene",
                    PathJoinSubstitution([pkg_share, "test_resources", "scene_info.xml"]),
                    "--publish_topic",
                    "/mujoco_robot_description",
                ],
                condition=IfCondition(
                    AndSubstitution(
                        LaunchConfiguration("use_mjcf_from_topic"), NotSubstitution(LaunchConfiguration("use_pid"))
                    )
                ),
            ),
            Node(
                package="mujoco_ros2_control",
                executable="robot_description_to_mjcf.sh",
                output="both",
                emulate_tty=True,
                arguments=["--publish_topic", "/mujoco_robot_description"],
                condition=IfCondition(
                    AndSubstitution(LaunchConfiguration("use_mjcf_from_topic"), LaunchConfiguration("use_pid"))
                ),
            ),
        ]
    )

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description, {"use_sim_time": True}],
        )
    )

    parameters_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])

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

    # Add controller spawners
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

    # Refer https://github.com/ros-controls/mujoco_ros2_control?tab=readme-ov-file#joints
    use_pid = DeclareLaunchArgument(
        "use_pid", default_value="false", description="If we should use PID control to enable other control modes"
    )

    headless = DeclareLaunchArgument("headless", default_value="false", description="Run in headless mode")

    use_mjcf_from_topic = DeclareLaunchArgument(
        "use_mjcf_from_topic",
        default_value="false",
        description="When set to true, the MJCF is generated at runtime from URDF",
    )

    test_transmissions = DeclareLaunchArgument(
        "test_transmissions",
        default_value="false",
        description="When set to true, a transmission is added to the robot model for testing purposes",
    )

    return LaunchDescription(
        [
            use_pid,
            headless,
            use_mjcf_from_topic,
            test_transmissions,
            OpaqueFunction(function=launch_setup),
        ]
    )
