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
Test launch file that includes the demo launch file from mujoco_ros2_control_demos.
This allows tests to use the demo configurations with test-specific parameters.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description that includes the demo launch file."""

    # Declare arguments
    use_pid = DeclareLaunchArgument(
        "use_pid", default_value="false", description="If we should use PID control to enable other control modes"
    )

    headless = DeclareLaunchArgument(
        "headless", default_value="true", description="Run in headless mode (default true for tests)"
    )

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

    # Include the demo launch file
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("mujoco_ros2_control_demos"), "launch/demo.launch.py")
        ),
        launch_arguments={
            "use_pid": LaunchConfiguration("use_pid"),
            "headless": LaunchConfiguration("headless"),
            "use_mjcf_from_topic": LaunchConfiguration("use_mjcf_from_topic"),
            "test_transmissions": LaunchConfiguration("test_transmissions"),
        }.items(),
    )

    return LaunchDescription(
        [
            use_pid,
            headless,
            use_mjcf_from_topic,
            test_transmissions,
            demo_launch,
        ]
    )
