#!/usr/bin/env python3

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
Regression test: joint 'direct_joint' has both a same-named MuJoCo actuator and a SimpleTransmission
(mechanical_reduction 2.0). The transmission must win in both directions, so a joint commanded to
0.05 drives its actuator to 0.10 and reads back 0.05. See actuator_state_to_joint_state() and
joint_command_to_actuator_command() in mujoco_system_interface.cpp for why.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running, check_if_js_published, check_node_running
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from launch_testing_ros import WaitForTopics
import pytest
import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT = "direct_joint"
TARGET = 0.05
MECHANICAL_REDUCTION = 2.0
# The actuator is driven to the joint command scaled by the reduction.
EXPECTED_ACTUATOR = TARGET * MECHANICAL_REDUCTION
TOLERANCE = 0.002


@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mujoco_ros2_control_tests"),
                "launch/direct_match_transmission_test_launch.py",
            )
        ),
        launch_arguments={"headless": "true"}.items(),
    )

    return LaunchDescription([launch_include, KeepAliveProc(), ReadyToTest()])


class TestTransmissionAppliedWhenJointAlsoDirectlyMatched(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("direct_match_transmission_test_node")
        self._latest_js = None
        self._latest_actuator_js = None
        self.node.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.node.create_subscription(JointState, "/mujoco_actuators_states", self._actuator_state_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _joint_state_cb(self, msg):
        self._latest_js = msg

    def _actuator_state_cb(self, msg):
        self._latest_actuator_js = msg

    def spin_until(self, predicate, timeout=15.0, spin_period=0.05):
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=spin_period)
            if predicate():
                return True
        return False

    def get_joint_value(self, msg, field, joint_name):
        if msg is None or joint_name not in msg.name:
            return None
        return getattr(msg, field)[msg.name.index(joint_name)]

    def test_node_start(self):
        check_node_running(self.node, "robot_state_publisher")

    def test_clock(self):
        with WaitForTopics([("/clock", Clock)], timeout=10.0):
            print("/clock is receiving messages!")

    def test_joint_and_actuator_states_published(self):
        check_if_js_published("/joint_states", [JOINT])
        check_if_js_published("/mujoco_actuators_states", [JOINT])

    def test_transmission_is_applied_in_both_directions(self):
        check_controllers_running(self.node, ["joint_state_broadcaster", "position_controller"])

        pub = self.node.create_publisher(Float64MultiArray, "/position_controller/commands", 10)
        self.assertTrue(
            self.spin_until(lambda: pub.get_subscription_count() > 0, timeout=5.0),
            "Controller did not subscribe to commands",
        )

        command = Float64MultiArray()
        command.data = [TARGET]
        end_time = time.time() + 2.0
        while time.time() < end_time:
            pub.publish(command)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        # The command path applies the transmission: the actuator is driven to command * reduction.
        reached_actuator = self.spin_until(
            lambda: self.get_joint_value(self._latest_actuator_js, "position", JOINT) is not None
            and abs(self.get_joint_value(self._latest_actuator_js, "position", JOINT) - EXPECTED_ACTUATOR) < TOLERANCE,
            timeout=10.0,
        )
        actuator_position = self.get_joint_value(self._latest_actuator_js, "position", JOINT)
        self.assertTrue(
            reached_actuator,
            f"MuJoCo actuator '{JOINT}' is at {actuator_position}, expected {EXPECTED_ACTUATOR}",
        )

        # ...and the state path applies it too, so the round trip closes on the joint side.
        joint_position = self.get_joint_value(self._latest_js, "position", JOINT)
        self.assertIsNotNone(joint_position, "No joint state received")
        self.assertAlmostEqual(
            joint_position,
            TARGET,
            delta=TOLERANCE,
            msg=f"Joint state for '{JOINT}' is {joint_position}, expected {TARGET}",
        )
