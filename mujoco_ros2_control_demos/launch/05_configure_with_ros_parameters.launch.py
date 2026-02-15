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
Tutorial 1: Basic Robot Demo

This tutorial demonstrates how to set the MuJoCo simulation by parsing parameters via ROS parameters instead of using encoding them in the URDF.
It launches a two-link arm robot with position controllers using a pre-defined MJCF model.

Key concepts:
- Loading a pre-defined MJCF model from demo_resources/scenes/scene.xml via ROS parameters
- Basic ros2_control integration with MuJoCo
- Position control of joints via position_controller

Usage:
    ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py
    ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py headless:=true

Control the robot:
    ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]"
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


def launch_setup(context, *args, **kwargs):
    pkg_share = FindPackageShare("mujoco_ros2_control_demos")

    # Resolve package share path so we can create/load a simulation parameters template file
    pkg_share_path = pkg_share.perform(context)

    # Ensure a simulation parameters template exists in the package config folder.
    # If it does not, generate a conservative template that the ros2_control_node can load.
    simulation_params_path = os.path.join(pkg_share_path, "config", "simulation_parameters.yaml")


    # Read the (possibly newly created) template and replace the placeholder
    # ${MUJOCO_MODEL_PATH} with the actual model path used by this demo.
    # We write the final parameter file to a temporary file and pass that to ros2_control_node.
    with open(simulation_params_path, "r", encoding="utf-8") as f:
        sim_content = f.read()

    # Choose the model path used by this demo (non-PID scene)
    mujoco_model_path = os.path.join(pkg_share_path, "demo_resources", "scenes", "scene.xml")

    sim_content_filled = sim_content.replace("${MUJOCO_MODEL_PATH}", mujoco_model_path)

    # Create a temp file to hold the final parameters (ros launch expects a filesystem path)
    tmp_file = tempfile.NamedTemporaryFile(prefix="simulation_parameters_", suffix=".yaml", delete=False)
    try:
        tmp_file.write(sim_content_filled.encode("utf-8"))
        tmp_file.flush()
        simulation_params_file = tmp_file.name
    finally:
        tmp_file.close()

    # Build robot description using the shared URDF
    # Even though the headless is set to false here, the ROS parameter one is taken into account.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "demo_resources", "robot", "test_robot.urdf"]),
            " use_pid:=false",
            " headless:=false",
            " use_mjcf_from_topic:=false",
            " use_transmissions:=false",
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    parameters_file = PathJoinSubstitution([pkg_share, "config", "controllers.yaml"])
    # simulation_params_file now points to the temp file containing the filled template

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
                ParameterFile(simulation_params_file),
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

    return LaunchDescription(
        [
            headless,
            OpaqueFunction(function=launch_setup),
        ]
    )
