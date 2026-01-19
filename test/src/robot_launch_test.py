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
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState, Image, CameraInfo
from controller_manager_msgs.srv import ListHardwareInterfaces


# This function specifies the processes to be run for our test
def generate_test_description_common(use_pid="false", use_mjcf_from_topic="false", test_transmissions="false"):
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"
    os.environ["USE_MJCF_FROM_TOPIC"] = use_mjcf_from_topic
    os.environ["TEST_TRANSMISSIONS"] = test_transmissions

    if use_mjcf_from_topic == "true":
        # Setup the venv needed for the make_mjcf_from_robot_description node
        os.system(
            os.path.join(
                get_package_share_directory("mujoco_ros2_control"),
                "scripts/robot_description_to_mjcf.sh --install-only",
            )
        )

    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("mujoco_ros2_control"),
                "launch/test_robot.launch.py",
            )
        ),
        launch_arguments={
            "headless": "true",
            "use_pid": use_pid,
            "use_mjcf_from_topic": use_mjcf_from_topic,
            "test_transmissions": test_transmissions,
        }.items(),
    )

    return LaunchDescription([launch_include, KeepAliveProc(), ReadyToTest()])


@pytest.mark.rostest
def generate_test_description():
    return generate_test_description_common(use_pid="false")


class TestFixture(unittest.TestCase):

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

    def test_node_start(self, proc_output):
        check_node_running(self.node, "robot_state_publisher")

    def test_clock(self):
        topic_list = [("/clock", Clock)]
        with WaitForTopics(topic_list, timeout=10.0):
            print("/clock is receiving messages!")

    def test_check_if_msgs_published(self):
        check_if_js_published(
            "/joint_states",
            [
                "joint1",
                "joint2",
                "gripper_left_finger_joint",
                "gripper_right_finger_joint",
            ],
        )

    def test_check_if_mujoco_actuators_states_published(self):
        if os.environ.get("TEST_TRANSMISSIONS") != "true":
            check_if_js_published(
                "/mujoco_actuators_states",
                ["joint1", "joint2", "gripper_left_finger_joint", "gripper_right_finger_joint"],
            )
        else:
            check_if_js_published(
                "/mujoco_actuators_states",
                ["actuator1", "actuator2", "gripper_left_finger_joint", "gripper_right_finger_joint"],
            )

    def test_arm(self):

        # Check if the controllers are running
        cnames = ["gripper_controller", "position_controller", "joint_state_broadcaster"]
        check_controllers_running(self.node, cnames)

        # Create a publisher to send commands to the position controller
        pub = self.node.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        # Wait for subscriber to connect
        end_time = time.time() + 10
        while time.time() < end_time:
            if pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)

        self.assertGreater(pub.get_subscription_count(), 0, "Controller did not subscribe to commands")

        msg = Float64MultiArray()
        msg.data = [0.5, -0.5]
        # This is needed to account for any missing message subscriptions
        end_time = time.time() + 2
        while time.time() < end_time:
            pub.publish(msg)
            time.sleep(0.1)

        # Allow some time for the message to be processed
        time.sleep(4.0)
        if os.environ.get("TEST_TRANSMISSIONS") == "true":
            # wait a bit more for the mujoco actuator states to be reach as they move double
            time.sleep(2.0)

        # Now, check that the joint states have been updated accordingly
        joint_state_topic = "/joint_states"
        wait_for_topics = WaitForTopics([(joint_state_topic, JointState)], timeout=20.0)
        assert wait_for_topics.wait(), f"Topic '{joint_state_topic}' not found!"
        msgs = wait_for_topics.received_messages(joint_state_topic)
        msg = msgs[0]
        assert len(msg.name) >= 2, "Joint states message doesn't have 2 joints"
        joint1_index = msg.name.index("joint1")
        joint2_index = msg.name.index("joint2")
        self.assertAlmostEqual(
            msg.position[joint1_index], 0.5, delta=0.05, msg="joint1 did not reach the commanded position"
        )
        self.assertAlmostEqual(
            msg.position[joint2_index], -0.5, delta=0.05, msg="joint2 did not reach the commanded position"
        )

        # MuJoCo actuator state
        actuator_state_topic = "/mujoco_actuators_states"
        wait_for_topics = WaitForTopics([(actuator_state_topic, JointState)], timeout=20.0)
        assert wait_for_topics.wait(), f"Topic '{actuator_state_topic}' not found!"
        msgs = wait_for_topics.received_messages(actuator_state_topic)
        msg = msgs[0]
        assert len(msg.name) >= 2, "MuJoCo actuator state message doesn't have 2 actuators"
        if os.environ.get("TEST_TRANSMISSIONS") != "true":
            actuator1_index = msg.name.index("joint1")
            actuator2_index = msg.name.index("joint2")
            actuator1_reduction = 1.0
            actuator2_reduction = 1.0
        else:
            actuator1_index = msg.name.index("actuator1")
            actuator2_index = msg.name.index("actuator2")
            actuator1_reduction = 2.0
            actuator2_reduction = 0.5
        self.assertAlmostEqual(
            msg.position[actuator1_index],
            0.5 * actuator1_reduction,
            delta=0.05,
            msg="actuator1 did not reach the commanded position",
        )
        self.assertAlmostEqual(
            msg.position[actuator2_index],
            -0.5 * actuator2_reduction,
            delta=0.05,
            msg="actuator2 did not reach the commanded position",
        )
        wait_for_topics.shutdown()

    def test_gripper(self):

        # Check if the controllers are running
        cnames = ["gripper_controller", "position_controller", "joint_state_broadcaster"]
        check_controllers_running(self.node, cnames)

        # Create a publisher to send commands to the gripper controller
        pub = self.node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

        # Wait for subscriber to connect
        end_time = time.time() + 10
        while time.time() < end_time:
            if pub.get_subscription_count() > 0:
                break
            time.sleep(0.1)

        self.assertGreater(pub.get_subscription_count(), 0, "Controller did not subscribe to commands")

        msg = Float64MultiArray()
        msg.data = [-0.04]  # Close gripper
        end_time = time.time() + 1.0
        while time.time() < end_time:
            pub.publish(msg)
            time.sleep(0.1)

        # Allow some time for the message to be processed
        time.sleep(4.0)

        # Now, check that the joint states have been updated accordingly
        joint_state_topic = "/joint_states"
        wait_for_topics = WaitForTopics([(joint_state_topic, JointState)], timeout=20.0)
        assert wait_for_topics.wait(), f"Topic '{joint_state_topic}' not found!"
        msgs = wait_for_topics.received_messages(joint_state_topic)
        msg = msgs[0]
        assert "gripper_left_finger_joint" in msg.name, "gripper_left_finger_joint not found in joint states"
        assert "gripper_right_finger_joint" in msg.name, "gripper_right_finger_joint not found in joint states"
        left_index = msg.name.index("gripper_left_finger_joint")
        right_index = msg.name.index("gripper_right_finger_joint")
        self.assertAlmostEqual(
            msg.position[left_index],
            -0.04,
            delta=0.005,
            msg="gripper_left_finger_joint did not reach commanded position",
        )
        self.assertAlmostEqual(
            msg.position[right_index],
            0.04,
            delta=0.005,
            msg="gripper_right_finger_joint did not reach commanded position",
        )
        wait_for_topics.shutdown()

        # Verify the same in the mujoco_actuators_states topic
        actuator_state_topic = "/mujoco_actuators_states"
        wait_for_topics = WaitForTopics([(actuator_state_topic, JointState)], timeout=20.0)
        assert wait_for_topics.wait(), f"Topic '{actuator_state_topic}' not found!"
        msgs = wait_for_topics.received_messages(actuator_state_topic)
        msg = msgs[0]
        assert "gripper_left_finger_joint" in msg.name, "gripper_left_finger_joint not found in actuator states"
        assert "gripper_right_finger_joint" in msg.name, "gripper_right_finger_joint not found in actuator states"
        left_index = msg.name.index("gripper_left_finger_joint")
        right_index = msg.name.index("gripper_right_finger_joint")
        self.assertAlmostEqual(
            msg.position[left_index],
            -0.04,
            delta=0.005,
            msg="gripper_left_finger_joint did not reach commanded position",
        )
        self.assertAlmostEqual(
            msg.position[right_index],
            0.04,
            delta=0.005,
            msg="gripper_right_finger_joint did not reach commanded position",
        )
        wait_for_topics.shutdown()

    # Runs the tests when the DISPLAY is set
    @unittest.skipIf(os.environ.get("DISPLAY", "") == "", "Skipping camera tests in headless mode.")
    def test_camera_topics(self):
        topic_list = [
            ("/camera/color/image_raw", Image),
            ("/camera/color/camera_info", CameraInfo),
            ("/camera/aligned_depth_to_color/image_raw", Image),
        ]
        wait_for_topics = WaitForTopics(topic_list, timeout=5.0)
        assert wait_for_topics.wait(), "Not all camera topics were received in time!"
        assert wait_for_topics.topics_not_received() == set(), "Some topics were not received!"
        assert wait_for_topics.topics_received() == {t[0] for t in topic_list}, "Not all topics were received!"
        wait_for_topics.shutdown()


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
        expected_command_interfaces = ["joint1/position", "joint2/position", "gripper_left_finger_joint/position"]

        assert len(available_command_interfaces_names) == len(expected_command_interfaces), (
            f"Expected {len(expected_command_interfaces)} command interfaces, "
            f"got {len(available_command_interfaces_names)}"
        )
        assert set(available_command_interfaces_names) == set(
            expected_command_interfaces
        ), f"Command interfaces do not match expected. Got: {available_command_interfaces_names}"

        self.node.get_logger().info("Available hardware interfaces check passed.")


class TestMJCFGenerationFromURDF(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if os.environ.get("USE_MJCF_FROM_TOPIC") != "true":
            raise unittest.SkipTest("Skipping MJCF generation tests because use_mjcf_from_topic is not true")
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def tearDown(self):
        self.node.destroy_node()

    def test_check_for_mujoco_robot_description_topic(self):
        # Create a QoS profile for transient_local topics
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        received_msgs = []

        def callback(msg):
            received_msgs.append(msg)

        sub = self.node.create_subscription(String, "/mujoco_robot_description", callback, qos_profile)

        end_time = time.time() + 15.0
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if received_msgs:
                break

        assert received_msgs, "The MuJoCo robot description topic is not published"
        msg = received_msgs[0]
        assert "<mujoco" in msg.data, "The MuJoCo robot description does not contain expected content"
        self.node.destroy_subscription(sub)
