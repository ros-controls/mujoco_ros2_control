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
Tutorial 5: Base Velocity Plugin

This tutorial demonstrates driving a mobile/floating-base robot with BaseVelocityPlugin.
The mobile base is a free-floating, wheeled chassis (MJCF <freejoint>) carrying a
Because the override is kinematic, it outranks the physics engine: the mounted arm's
reaction forces as it swings have no effect on the base's velocity, but neither does
colliding with the wall in the scene -- see doc/tutorials.rst (Tutorial 5) and
mobile_base.xml for the full rationale.

Key concepts:
- BaseVelocityPlugin: hard free-joint velocity override, applied by writing directly into
  data->qvel during update()
- Driving a MJCF <freejoint> body from a cmd_vel-style topic
- Disturbance immunity: the override holds the commanded velocity (zero, by default)
- Floating-base odometry publishing (mujoco_ros2_control's odom_free_joint_name)

Resources used:
- demo_resources/scenes/scene_mobile_base.xml (scene with mobile base + wall obstacle)
- demo_resources/mobile_base/mobile_base.xml (MJCF chassis+arm with wheels and freejoint)
- demo_resources/mobile_base/mobile_base.urdf (URDF with the arm's ros2_control joint)
- config/mujoco_ros2_control_plugins_base_velocity.yaml (BaseVelocityPlugin configuration)
- config/controllers_base_velocity.yaml (joint_state_broadcaster + arm_position_controller)

Usage:
    ros2 launch mujoco_ros2_control_demos 05_base_velocity_plugin.launch.py
    ros2 launch mujoco_ros2_control_demos 05_base_velocity_plugin.launch.py headless:=true

Drive the base:
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}" --rate 10

Swing the arm back and forth (with no cmd_vel active, the base should stay close to put):
    ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.7]"
    ros2 topic pub /arm_position_controller/commands std_msgs/msg/Float64MultiArray "data: [-0.7]"

Watch odometry:
    ros2 topic echo /simulator/floating_base_state
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

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "demo_resources", "mobile_base", "mobile_base.urdf"]),
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )

    robot_description_str = robot_description_content.perform(context)
    robot_description = {"robot_description": ParameterValue(value=robot_description_str, value_type=str)}

    controllers_file = PathJoinSubstitution([pkg_share, "config", "controllers_base_velocity.yaml"])
    mujoco_plugins_file = PathJoinSubstitution([pkg_share, "config", "mujoco_ros2_control_plugins_base_velocity.yaml"])

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

    # ros2_control node with MuJoCo. The base itself has no ros2_control joints --
    # BaseVelocityPlugin drives it directly from /cmd_vel -- only the mounted arm does.
    nodes.append(
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            emulate_tty=True,
            output="both",
            parameters=[
                {"use_sim_time": True},
                ParameterFile(controllers_file),
                ParameterFile(mujoco_plugins_file),
            ],
            remappings=(
                [("~/robot_description", "/robot_description")] if os.environ.get("ROS_DISTRO") == "humble" else []
            ),
            on_exit=Shutdown(),
        )
    )

    # Controller spawners for the arm joint
    controllers_to_spawn = ["joint_state_broadcaster", "arm_position_controller"]
    for controller in controllers_to_spawn:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--param-file", controllers_file],
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
