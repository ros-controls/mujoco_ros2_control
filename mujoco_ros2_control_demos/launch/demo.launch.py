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

"""
Main Demo Launcher

This launch file provides a unified interface to launch different demo configurations.
Based on the arguments provided, it delegates to the appropriate specialized launch file:

- 01_basic_robot.launch.py: Basic robot with position control (default)
- 02_mjcf_generation.launch.py: Runtime MJCF generation from URDF
- 03_pid_control.launch.py: PID-based control for velocity/effort modes
- 04_transmissions.launch.py: Transmission interface demo

Usage:
    # Basic robot demo (default)
    ros2 launch mujoco_ros2_control_demos demo.launch.py

    # PID control demo
    ros2 launch mujoco_ros2_control_demos demo.launch.py use_pid:=true

    # MJCF generation demo
    ros2 launch mujoco_ros2_control_demos demo.launch.py use_mjcf_from_topic:=true

    # Transmissions demo
    ros2 launch mujoco_ros2_control_demos demo.launch.py test_transmissions:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("mujoco_ros2_control_demos")

    # Get argument values
    use_pid = LaunchConfiguration("use_pid").perform(context) == "true"
    use_mjcf_from_topic = LaunchConfiguration("use_mjcf_from_topic").perform(context) == "true"
    test_transmissions = LaunchConfiguration("test_transmissions").perform(context) == "true"
    headless = LaunchConfiguration("headless").perform(context)

    # Determine which launch file to use based on arguments
    if test_transmissions:
        launch_file = "04_transmissions.launch.py"
        launch_args = {"headless": headless, "use_pid": "true" if use_pid else "false"}
    elif use_mjcf_from_topic:
        launch_file = "02_mjcf_generation.launch.py"
        launch_args = {"headless": headless, "use_urdf_inputs": "true" if use_pid else "false"}
    elif use_pid:
        launch_file = "03_pid_control.launch.py"
        launch_args = {"headless": headless}
    else:
        launch_file = "01_basic_robot.launch.py"
        launch_args = {"headless": headless}

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_share, "launch", launch_file])),
            launch_arguments=launch_args.items(),
        )
    ]


def generate_launch_description():
    use_pid = DeclareLaunchArgument(
        "use_pid",
        default_value="false",
        description="If true, uses PID control (03_pid_control.launch.py)",
    )

    headless = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run simulation without visualization window",
    )

    use_mjcf_from_topic = DeclareLaunchArgument(
        "use_mjcf_from_topic",
        default_value="false",
        description="If true, generates MJCF at runtime (02_mjcf_generation.launch.py)",
    )

    test_transmissions = DeclareLaunchArgument(
        "test_transmissions",
        default_value="false",
        description="If true, uses transmissions demo (04_transmissions.launch.py)",
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
