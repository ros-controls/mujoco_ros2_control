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

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running, check_if_js_published, check_node_running
from controller_manager_msgs.srv import ListHardwareInterfaces
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


@pytest.mark.rostest
def generate_test_description():
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mujoco_ros2_control_tests"),
                "launch/site_velocity_test_launch.py",
            )
        ),
        launch_arguments={"headless": "true"}.items(),
    )

    return LaunchDescription([launch_include, KeepAliveProc(), ReadyToTest()])


class TestSiteVelocityActuator(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("site_velocity_test_node")
        self._latest_js = None
        self._latest_actuator_js = None
        self._js_sub = self.node.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self._actuator_sub = self.node.create_subscription(
            JointState, "/mujoco_actuators_states", self._actuator_state_cb, 10
        )

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
        check_if_js_published("/joint_states", ["slider_joint"])
        check_if_js_published("/mujoco_actuators_states", ["slider_joint"])

    def test_available_hardware_interfaces(self):
        client = self.node.create_client(ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))

        future = client.call_async(ListHardwareInterfaces.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result())

        response = future.result()
        state_interfaces = {iface.name for iface in response.state_interfaces}
        command_interfaces = {iface.name for iface in response.command_interfaces}

        self.assertEqual(
            state_interfaces,
            {
                "slider_joint/position",
                "slider_joint/velocity",
                "slider_joint/effort",
            },
        )
        self.assertEqual(command_interfaces, {"slider_joint/velocity"})

    def test_site_velocity_actuator_moves_slider(self):
        check_controllers_running(self.node, ["joint_state_broadcaster", "site_velocity_controller"])

        pub = self.node.create_publisher(Float64MultiArray, "/site_velocity_controller/commands", 10)
        self.assertTrue(
            self.spin_until(lambda: pub.get_subscription_count() > 0, timeout=5.0),
            "Controller did not subscribe to commands",
        )

        self.assertTrue(
            self.spin_until(lambda: self.get_joint_value(self._latest_js, "position", "slider_joint") is not None),
            "No slider joint state received",
        )
        start_position = self.get_joint_value(self._latest_js, "position", "slider_joint")

        command = Float64MultiArray()
        command.data = [0.5]
        pub.publish(command)

        self.assertTrue(
            self.spin_until(
                lambda: self.get_joint_value(self._latest_js, "position", "slider_joint") is not None
                and self.get_joint_value(self._latest_js, "position", "slider_joint") > start_position + 0.1,
                timeout=1.0,
            ),
            "Slider joint did not move sufficiently after commanding the site velocity actuator",
        )

        self.assertTrue(
            self.spin_until(
                lambda: self.get_joint_value(self._latest_actuator_js, "velocity", "slider_joint") is not None
                and self.get_joint_value(self._latest_actuator_js, "velocity", "slider_joint") > 0.4,
                timeout=2.0,
            ),
            "Actuator state did not report sufficiently positive slider velocity",
        )
