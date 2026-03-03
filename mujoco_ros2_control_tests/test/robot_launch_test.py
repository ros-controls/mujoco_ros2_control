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
from mujoco_ros2_control_msgs.srv import ResetWorld
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState, Image, CameraInfo
from controller_manager_msgs.srv import ListHardwareInterfaces, SwitchController


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
                get_package_share_directory("mujoco_ros2_control_tests"),
                "launch/test_launch.py",
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
        self._latest_js = None
        self._latest_actuator_js = None
        self._js_sub = self.node.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)
        self._actuator_sub = self.node.create_subscription(
            JointState, "/mujoco_actuators_states", self.actuator_joint_state_cb, 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def joint_state_cb(self, msg):
        self._latest_js = msg

    def actuator_joint_state_cb(self, msg):
        self._latest_actuator_js = msg

    def spin_until(self, predicate, timeout=20.0, spin_period=0.05):
        """Spin the node until predicate() returns True or timeout is reached.

        Returns True if the predicate was satisfied, False on timeout.
        """
        end_time = time.time() + timeout
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=spin_period)
            if predicate():
                return True
        return False

    def _check_positions(self, msg, expected_positions, delta):
        """Return True if all expected joints are within delta in msg."""
        if msg is None:
            return False
        try:
            return all(abs(msg.position[msg.name.index(jn)] - ep) <= delta for jn, ep in expected_positions.items())
        except ValueError:
            return False

    def check_controllers_running_with_retry(self, controller_names, timeout=15.0):
        """Calls check_controllers_running with a retry to give the check more time to succeed."""

        def call_check_controllers_running():
            try:
                check_controllers_running(self.node, controller_names)
                return True
            except Exception:
                return False

        if not self.spin_until(call_check_controllers_running, timeout=timeout):
            self.fail(f"Controllers not running after {timeout}s: {controller_names}")

    def wait_for_joint_positions(
        self, expected_positions, delta=0.05, timeout=15.0, verify_efforts=True, topic="joint_states"
    ):
        """Helper function to poll until the joint states reach the desired position."""

        # Get the relevant joint state message, which we do just by checking the topic name...
        if topic == "joint_states":
            get_msg = lambda: self._latest_js  # noqa: E731
        else:
            get_msg = lambda: self._latest_actuator_js  # noqa: E731

        ok = self.spin_until(
            lambda: self._check_positions(get_msg(), expected_positions, delta),
            timeout=timeout,
        )

        msg = get_msg()
        if not ok:
            # Try to get details about where exactly we failed, sometimes nothing has updated
            details = {}
            if msg is not None and msg.name:
                for jn in expected_positions:
                    if jn in msg.name:
                        details[jn] = msg.position[msg.name.index(jn)]
                    else:
                        details[jn] = "NOT FOUND"
            self.fail(
                f"joints did not reach the commanded position in {timeout}s. "
                f"expected: {expected_positions}, checked: {details}"
            )

        if verify_efforts and msg is not None:
            # make sure the efforts field is non-zero (indicating PID/ effort reporting is working)
            self.assertTrue(
                any(abs(effort) > 1e-6 for effort in msg.effort),
                "Effort field is zero, PID/Effort reporting may not be working",
            )

    def verify_arm_joint_states(self, expected_positions, delta=0.05, verify_efforts=True):
        self.wait_for_joint_positions(
            expected_positions, delta=delta, verify_efforts=verify_efforts, topic="joint_states"
        )

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
        self.check_controllers_running_with_retry(cnames)

        # Create a publisher to send commands to the position controller
        pub = self.node.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        # Wait for subscriber to connect
        self.assertTrue(
            self.spin_until(lambda: pub.get_subscription_count() > 0, timeout=5.0),
            "Controller did not subscribe to commands",
        )

        msg = Float64MultiArray()
        msg.data = [0.5, -0.5]
        # This is needed to account for any missing message subscriptions
        end_time = time.time() + 2
        while time.time() < end_time:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Wait for joints to reach target positions
        self.wait_for_joint_positions({"joint1": 0.5, "joint2": -0.5}, delta=0.05, timeout=5.0)

        if os.environ.get("TEST_TRANSMISSIONS") != "true":
            expected_actuators = {"joint1": 0.5, "joint2": -0.5}
        else:
            expected_actuators = {"actuator1": 0.5 * 2.0, "actuator2": -0.5 * 0.5}

        self.wait_for_joint_positions(expected_actuators, delta=0.05, timeout=5.0, topic="actuator_states")

        # make sure the efforts field is non-zero (indicating PID/ effort reporting is working)
        self.assertTrue(
            self._latest_actuator_js is not None and any(abs(e) > 1e-6 for e in self._latest_actuator_js.effort),
            "Effort field is zero, PID/Effort reporting may not be working",
        )

    def test_gripper(self):

        # Check if the controllers are running
        cnames = ["gripper_controller", "position_controller", "joint_state_broadcaster"]
        self.check_controllers_running_with_retry(cnames)

        # Create a publisher to send commands to the gripper controller
        pub = self.node.create_publisher(Float64MultiArray, "/gripper_controller/commands", 10)

        # Wait for subscriber to connect
        self.assertTrue(
            self.spin_until(lambda: pub.get_subscription_count() > 0, timeout=5.0),
            "Controller did not subscribe to commands",
        )

        msg = Float64MultiArray()
        msg.data = [-0.04]  # Close gripper
        end_time = time.time() + 1.0
        while time.time() < end_time:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Wait for the gripper to get to the target pose
        self.wait_for_joint_positions(
            {"gripper_left_finger_joint": -0.04, "gripper_right_finger_joint": 0.04},
            delta=0.005,
            timeout=5.0,
        )

        # And confirm that the mujoco_actuators_states also gets there
        self.wait_for_joint_positions(
            {"gripper_left_finger_joint": -0.04, "gripper_right_finger_joint": 0.04},
            delta=0.005,
            timeout=5.0,
            topic="actuator_states",
        )

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

    def test_reset_world_service(self):
        """Test that the reset_world service resets robot to initial position."""
        # Check if controllers are running
        cnames = ["gripper_controller", "position_controller", "joint_state_broadcaster"]
        self.check_controllers_running_with_retry(cnames)

        # Create a publisher to send commands
        pub = self.node.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

        # Wait for subscriber to connect
        self.assertTrue(
            self.spin_until(lambda: pub.get_subscription_count() > 0, timeout=10.0),
            "Controller did not subscribe to commands",
        )

        # Send a command to move the joints to a different position
        msg = Float64MultiArray()
        msg.data = [-0.5, 0.5]

        # Publish commands
        end_time = time.time() + 2
        while time.time() < end_time:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Poll until joints converge (replaces time.sleep + verify)
        self.wait_for_joint_positions({"joint1": -0.5, "joint2": 0.5}, delta=0.05, timeout=15.0)

        # Deactivate the position controller before reset
        switch_client = self.node.create_client(SwitchController, "/controller_manager/switch_controller")
        if not switch_client.wait_for_service(timeout_sec=10.0):
            self.fail("switch_controller service not available")

        # Deactivate position_controller
        switch_request = SwitchController.Request()
        switch_request.deactivate_controllers = ["position_controller"]
        switch_request.strictness = SwitchController.Request.BEST_EFFORT
        future = switch_client.call_async(switch_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().ok, "Failed to deactivate position_controller")
        self.node.get_logger().info("position_controller deactivated")

        # Record clock time before reset
        clock_topic = "/clock"
        wait_for_clock = WaitForTopics([(clock_topic, Clock)], timeout=20.0)
        assert wait_for_clock.wait(), f"Topic '{clock_topic}' not found!"
        clock_msgs = wait_for_clock.received_messages(clock_topic)
        clock_before_reset = clock_msgs[-1]

        self.node.get_logger().info(f"Clock before reset: {clock_before_reset}")

        # Now call the reset_world service
        reset_client = self.node.create_client(ResetWorld, "/mujoco_ros2_control_node/reset_world")

        # Wait for service to be available
        if not reset_client.wait_for_service(timeout_sec=10.0):
            self.fail("reset_world service not available")

        # Call the reset service
        request = ResetWorld.Request()
        future = reset_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        if future.result() is None:
            self.fail("reset_world service call failed")

        self.node.get_logger().info("reset_world service called successfully")

        # Verify clock was NOT reset (should still be greater than before reset)
        rclpy.spin_once(self.node, timeout_sec=0.1)
        clock_msgs_after_reset = wait_for_clock.received_messages(clock_topic)
        clock_after_reset = clock_msgs_after_reset[-1]
        time_pre_reset = clock_before_reset.clock.sec + clock_before_reset.clock.nanosec * 1e-9
        time_post_reset = clock_after_reset.clock.sec + clock_after_reset.clock.nanosec * 1e-9
        self.assertGreaterEqual(
            time_post_reset,
            time_pre_reset,
            f"Clock was reset! Before: {clock_before_reset}, After: {clock_after_reset}",
        )
        self.node.get_logger().info("Clock continuity verified - clock was NOT reset")

        # Poll until joints return to zero (replaces time.sleep + verify)
        self.wait_for_joint_positions({"joint1": 0.0, "joint2": 0.0}, delta=0.05, timeout=15.0, verify_efforts=False)

        # Reactivate the position controller
        switch_request = SwitchController.Request()
        switch_request.activate_controllers = ["position_controller"]
        switch_request.strictness = SwitchController.Request.BEST_EFFORT
        future = switch_client.call_async(switch_request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().ok, "Failed to reactivate position_controller")
        self.node.get_logger().info("position_controller reactivated")

        # Send the command again to verify controller still works after reset
        end_time = time.time() + 2
        while time.time() < end_time:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)

        # Poll until joints converge again
        self.wait_for_joint_positions({"joint1": -0.5, "joint2": 0.5}, delta=0.05, timeout=15.0)
        wait_for_clock.shutdown()


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
