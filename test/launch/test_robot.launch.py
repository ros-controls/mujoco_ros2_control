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
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mujoco_ros2_control"),
                    "test_resources",
                    "test_robot.urdf",
                ]
            ),
            " ",
            "use_pid:=",
            LaunchConfiguration("use_pid"),
            " ",
            "headless:=",
            LaunchConfiguration("headless"),
            " ",
            "use_mjcf_from_topic:=",
            LaunchConfiguration("use_mjcf_from_topic"),
            " ",
            "use_transmissions:=",
            LaunchConfiguration("test_transmissions"),
        ]
    )

    robot_description_str = robot_description_content.perform(context)

    # Detect the content of the mujoco_model parameter in the robot_description string
    # <param name="mujoco_model">$(find mujoco_ros2_control)/test_resources/scene.xml</param>
    if LaunchConfiguration("test_transmissions").perform(context) == "true":
        mujoco_model_path = None
        pids_config_file = None

        if "mujoco_model" in robot_description_str:
            mujoco_model_path = robot_description_str.split('mujoco_model">')[1].split("</param>")[0].strip()

        if "pids_config_file" in robot_description_str:
            pids_config_file = robot_description_str.split('pids_config_file">')[1].split("</param>")[0].strip()

        # Now load the file and replace the joint1 and joint2 with actuator1 and actuator2
        if mujoco_model_path is not None:
            with open(mujoco_model_path) as file:
                scene_content = file.read()

            # If there is any include, then just copy that file to the /tmp too
            # <include file="test_robot.xml"/>
            if '<include file="' in scene_content:
                include_file = scene_content.split('<include file="')[1].split('"/>')[0].strip()
                # This include path is relative to the mujoco_model_path
                include_file_path = os.path.join(os.path.dirname(mujoco_model_path), include_file)

                with open(include_file_path) as include_file_handle:
                    include_content = include_file_handle.read()

                # Replace joint1 and joint2 with actuator1 and actuator2
                include_content = include_content.replace("joint1", "actuator1")
                include_content = include_content.replace("joint2", "actuator2")

                # Copy to temp
                temp_include_file = tempfile.NamedTemporaryFile(delete=False, suffix=".xml")
                temp_include_file.write(include_content.encode())
                temp_include_file.close()

                # Replace include file path in scene_content
                scene_content = scene_content.replace(include_file, temp_include_file.name)

            # Write to a temporary file
            temp_scene_file = tempfile.NamedTemporaryFile(delete=False, suffix=".xml")
            temp_scene_file.write(scene_content.encode())
            temp_scene_file.close()

            if pids_config_file:
                with open(pids_config_file) as file:
                    pids_content = file.read()

                # Replace joint1 and joint2 with actuator1 and actuator2
                pids_content = pids_content.replace("joint1", "actuator1")
                pids_content = pids_content.replace("joint2", "actuator2")

                # Write to a temporary file
                temp_pids_file = tempfile.NamedTemporaryFile(delete=False, suffix=".yaml")
                temp_pids_file.write(pids_content.encode())
                temp_pids_file.close()

                # Update the robot_description_content to point to the new pids file
                robot_description_str = robot_description_str.replace(pids_config_file, temp_pids_file.name)

            # Update the robot_description_content to point to the new scene file
            robot_description_str = robot_description_str.replace(mujoco_model_path, temp_scene_file.name)
            print("Modified scene file with transmissions at:", temp_scene_file.name)
            print(robot_description_str)

    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    converter_arguments_no_pid = [
        "--robot_description",
        robot_description_str,
        "--m",
        PathJoinSubstitution(
            [
                FindPackageShare("mujoco_ros2_control"),
                "test_resources",
                "test_inputs.xml",
            ]
        ),
        "--scene",
        PathJoinSubstitution(
            [
                FindPackageShare("mujoco_ros2_control"),
                "test_resources",
                "scene_info.xml",
            ]
        ),
        "--publish_topic",
        "/mujoco_robot_description",
    ]

    converter_arguments_pid = [
        "--publish_topic",
        "/mujoco_robot_description",
    ]

    converter_node_no_pid = Node(
        package="mujoco_ros2_control",
        executable="robot_description_to_mjcf.sh",
        output="both",
        emulate_tty=True,
        arguments=converter_arguments_no_pid,
        condition=UnlessCondition(LaunchConfiguration("use_pid")),
    )

    converter_node_pid = Node(
        package="mujoco_ros2_control",
        executable="robot_description_to_mjcf.sh",
        output="both",
        emulate_tty=True,
        arguments=converter_arguments_pid,
        condition=IfCondition(LaunchConfiguration("use_pid")),
    )

    controller_parameters = ParameterFile(
        PathJoinSubstitution([FindPackageShare("mujoco_ros2_control"), "config", "controllers.yaml"]),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters,
        ],
        on_exit=Shutdown(),
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster",
        ],
        output="both",
    )

    spawn_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_position_controller",
        arguments=[
            "position_controller",
        ],
        output="both",
    )

    return [
        robot_state_publisher_node,
        control_node,
        spawn_joint_state_broadcaster,
        spawn_position_controller,
        converter_node_pid,
        converter_node_no_pid,
    ]


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
