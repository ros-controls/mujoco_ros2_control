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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Refer https://github.com/ros-controls/mujoco_ros2_simulation?tab=readme-ov-file#joints
    use_pid = DeclareLaunchArgument(
        "use_pid", default_value="false", description="If we should use PID control to enable other control modes"
    )

    headless = DeclareLaunchArgument("headless", default_value="false", description="Run in headless mode")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mujoco_ros2_simulation"),
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
        ]
    )

    robot_description = {"robot_description": ParameterValue(value=robot_description_content, value_type=str)}

    controller_parameters = ParameterFile(
        PathJoinSubstitution([FindPackageShare("mujoco_ros2_simulation"), "config", "controllers.yaml"]),
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
        package="mujoco_ros2_simulation",
        executable="ros2_control_node",
        output="both",
        parameters=[
            {"use_sim_time": True},
            controller_parameters,
        ],
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

    return LaunchDescription(
        [
            use_pid,
            headless,
            robot_state_publisher_node,
            control_node,
            spawn_joint_state_broadcaster,
            spawn_position_controller,
        ]
    )
