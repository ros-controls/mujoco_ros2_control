#!/usr/bin/env python3

# Copyright 2025 PAL Robotics S.L.
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

import sys
import os
import rclpy
import pytest
import unittest

from controller_manager_msgs.srv import ListHardwareInterfaces

sys.path.append(os.path.dirname(__file__))  # noqa: E402

from robot_launch_test import generate_test_description_common  # noqa: E402
from robot_launch_test import TestFixture  # noqa: F401, E402


@pytest.mark.rostest
def generate_test_description():
    return generate_test_description_common(use_pid="true")


class TestFixtureHardwareInterfacesCheck(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_available_hardware_interfaces(self):
        # Call /controller_manager/list_hardware_interfaces service and check the response
        client = self.node.create_client(ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces")
        if not client.wait_for_service(timeout_sec=10.0):
            self.fail("Service /controller_manager/list_hardware_interfaces not available")

        request = ListHardwareInterfaces.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        if future.result() is None:
            self.fail("Service call to /controller_manager/list_hardware_interfaces failed")
        response = future.result()

        # available state interfaces
        available_state_interfaces_names = [iface.name for iface in response.state_interfaces]
        expected_state_interfaces = [
            "gripper_left_finger_joint/position",
            "gripper_left_finger_joint/velocity",
            "gripper_left_finger_joint/effort",
            "gripper_left_finger_joint/force",
            "gripper_right_finger_joint/position",
            "gripper_right_finger_joint/velocity",
            "gripper_right_finger_joint/effort",
            "gripper_right_finger_joint/force",
            "joint1/position",
            "joint1/velocity",
            "joint1/effort",
            "joint1/torque",
            "joint2/position",
            "joint2/velocity",
            "joint2/effort",
            "joint2/torque",
        ]
        assert len(available_state_interfaces_names) == len(
            expected_state_interfaces
        ), f"Expected {len(expected_state_interfaces)} state interfaces, got {len(available_state_interfaces_names)}"
        assert set(available_state_interfaces_names) == set(
            expected_state_interfaces
        ), f"State interfaces do not match expected. Got: {available_state_interfaces_names}"

        # available command interfaces
        available_command_interfaces_names = [iface.name for iface in response.command_interfaces]
        expected_command_interfaces = [
            "joint1/position",
            "joint1/velocity",
            "joint2/position",
            "joint2/velocity",
            "gripper_left_finger_joint/position",
        ]
        assert len(available_command_interfaces_names) == len(expected_command_interfaces), (
            f"Expected {len(expected_command_interfaces)} command interfaces, "
            f"got {len(available_command_interfaces_names)}"
        )
        assert set(available_command_interfaces_names) == set(
            expected_command_interfaces
        ), f"Command interfaces do not match expected. Got: {available_command_interfaces_names}"

        self.node.get_logger().info("Available hardware interfaces check passed.")
