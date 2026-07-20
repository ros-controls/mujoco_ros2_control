/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <thread>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include <mujoco_ros2_control/mujoco_simulation.hpp>
#include <mujoco_ros2_control_msgs/msg/free_joint_state.hpp>
#include <mujoco_ros2_control_msgs/srv/set_free_joint_state.hpp>

namespace
{

// Basic model for executing unit tests, has a hinge joint with an actuator, plus two
// free-floating bodies ("free_object", "free_object_2") used to exercise the free-joint
// reset service (including its list/batch behaviour). Most importantly:
//    nu=1, nq=15 (1 hinge + 7 + 7 free joint), nv=13 (1 hinge + 6 + 6 free joint), nbody=4
//    (world + pendulum + free_object + free_object_2)
constexpr const char* kTestModel = R"(<?xml version="1.0"?>
<mujoco model="test_simulation">
  <option timestep="0.002"/>

  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="1"/>
    </body>
    <body name="free_object" pos="1 0 1">
      <freejoint name="free_object_joint"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
    <body name="free_object_2" pos="2 0 1">
      <freejoint name="free_object_2_joint"/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <position name="hinge_pos" joint="hinge" kp="10"/>
  </actuator>

  <keyframe>
    <key name="home" qpos="0.5 1 0 1 1 0 0 0 2 0 1 1 0 0 0"/>
  </keyframe>
</mujoco>
)";

// Write to disk for testing
const std::string kTestModelPath = "/tmp/test_mujoco_simulation_model.xml";
void write_test_model()
{
  std::ofstream file(kTestModelPath);
  file << kTestModel;
  file.close();
}

}  // namespace

class MujocoSimulationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    write_test_model();
    const auto* test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    const std::string node_name = std::string("test_sim_") + test_info->name();
    node_ = std::make_shared<rclcpp::Node>(node_name);
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    sim_ = std::make_unique<mujoco_ros2_control::MujocoSimulation>();
  }

  void TearDown() override
  {
    // Clean up the executor to kill callbacks
    if (executor_)
    {
      executor_->cancel();
    }
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    // Then tear down the sim
    if (sim_)
    {
      sim_->shutdown();
      sim_.reset();
    }
    node_.reset();
    executor_.reset();

    // Clean up test files if present
    if (std::filesystem::exists(kTestModelPath))
    {
      std::filesystem::remove(kTestModelPath);
    }
  }

  // initialize the simulation in headless mode with default settings.
  bool initialize_sim()
  {
    return sim_->initialize(node_, kTestModelPath, "/mujoco_robot_description", -1.0, true);
  }

  // Helper function to poll a condition until it returns true or the timeout expires.
  bool wait_until(std::function<bool()> condition, std::chrono::milliseconds timeout = std::chrono::seconds(5),
                  std::chrono::milliseconds poll_interval = std::chrono::milliseconds(10))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (condition())
        return true;
      std::this_thread::sleep_for(poll_interval);
    }
    return condition();
  }

  // Helper function to confirm physics loop has stopped by verifying the time has "settled".
  // Verifies that the sim is continuing to step, but the time remains constant for 10 iterations.
  bool wait_for_pause()
  {
    uint64_t prev = sim_->step_count();
    int settled_count = 0;
    return wait_until([&]() {
      uint64_t now = sim_->step_count();
      if (now == prev)
      {
        settled_count++;
      }
      else
      {
        settled_count = 0;
      }
      prev = now;
      return settled_count >= 10;
    });
  };

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<mujoco_ros2_control::MujocoSimulation> sim_;
};

TEST_F(MujocoSimulationTest, TestInitialization)
{
  ASSERT_TRUE(initialize_sim());
  EXPECT_NE(sim_->model(), nullptr);
  EXPECT_NE(sim_->data(), nullptr);

  // nq/nv account for the "hinge" joint (1 each) plus the "free_object" and "free_object_2"
  // free joints (7 qpos each: xyz + wxyz quat; 6 qvel each: linear + angular).
  EXPECT_EQ(sim_->model()->nq, 15);
  EXPECT_EQ(sim_->model()->nv, 13);
  EXPECT_EQ(sim_->model()->nu, 1);
  EXPECT_EQ(sim_->model()->nbody, 4);
}

TEST_F(MujocoSimulationTest, ControlUpdateTests)
{
  ASSERT_TRUE(initialize_sim());

  // Simulate the read/write cycle in the ros2_control loop, making sure data
  // is updated where expected
  mjData* control = nullptr;
  sim_->copy_physics_data(control);
  ASSERT_NE(control, nullptr);

  // Update control inputs
  control->ctrl[0] = 0.75;
  control->qfrc_applied[0] = 1.5;

  // Apply to sim. Inputs are staged and copied into the sim data by the physics loop
  // immediately before each step, so they must not appear until stepping occurs.
  sim_->apply_control_data(control);
  EXPECT_DOUBLE_EQ(sim_->data()->ctrl[0], 0.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qfrc_applied[0], 0.0);

  // Once the physics loop starts stepping, the staged inputs should land in the sim data
  sim_->start_physics_thread();
  EXPECT_TRUE(wait_until([this]() { return sim_->data()->ctrl[0] == 0.75 && sim_->data()->qfrc_applied[0] == 1.5; }))
      << "Staged control inputs were not applied by the physics loop";

  mj_deleteData(control);
}

TEST_F(MujocoSimulationTest, XfrcAppliedTests)
{
  ASSERT_TRUE(initialize_sim());

  // Simulate the read/write cycle in the ros2_control loop, making sure data
  // is updated where expected
  mjData* control = nullptr;
  sim_->copy_physics_data(control);
  ASSERT_NE(control, nullptr);

  // Zero xfrc_applied (like for plugins), and apply a force
  const size_t body_id = 1;
  mju_zero(control->xfrc_applied, 6 * sim_->model()->nbody);
  control->xfrc_applied[body_id * 6 + 0] = 1.0;
  control->xfrc_applied[body_id * 6 + 1] = 2.0;
  control->xfrc_applied[body_id * 6 + 2] = 3.0;

  sim_->apply_control_data(control);

  // xfrc_applied should NOT be in mj_data_ directly, it goes through the triple
  // plugin buffer and gets composed in the physics loop
  EXPECT_DOUBLE_EQ(sim_->data()->xfrc_applied[body_id * 6 + 0], 0.0);
  EXPECT_DOUBLE_EQ(sim_->data()->xfrc_applied[body_id * 6 + 1], 0.0);
  EXPECT_DOUBLE_EQ(sim_->data()->xfrc_applied[body_id * 6 + 2], 0.0);

  mj_deleteData(control);
}

TEST_F(MujocoSimulationTest, PauseStepUnpause)
{
  ASSERT_TRUE(initialize_sim());
  sim_->start_physics_thread();

  // Confirm the sim has started
  ASSERT_TRUE(wait_until([this]() { return sim_->data()->time > 0.0; })) << "Simulation did not start stepping";

  // Setup service clients
  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto pause_client = node_->create_client<mujoco_ros2_control_msgs::srv::SetPause>(ns + "/set_pause");
  auto step_client = node_->create_client<mujoco_ros2_control_msgs::srv::StepSimulation>(ns + "/step_simulation");
  ASSERT_TRUE(pause_client->wait_for_service(std::chrono::seconds(5))) << "set_pause service not found";
  ASSERT_TRUE(step_client->wait_for_service(std::chrono::seconds(5))) << "step_simulation service not found";

  // Pause the simulation and wait for the service to return
  auto pause_req = std::make_shared<mujoco_ros2_control_msgs::srv::SetPause::Request>();
  pause_req->paused = true;
  auto pause_future = pause_client->async_send_request(pause_req);
  ASSERT_EQ(pause_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(pause_future.get()->success);

  // Once paused, time should not advance
  ASSERT_TRUE(wait_for_pause()) << "Simulation did not pause";
  const double time_after_pause = sim_->data()->time;
  ASSERT_TRUE(wait_until([&]() { return sim_->data()->time != time_after_pause; }, std::chrono::milliseconds(200)) ==
              false)
      << "Time should not advance while paused";

  // Next step the simulation by 10 steps
  const uint32_t num_steps = 10;
  auto step_req = std::make_shared<mujoco_ros2_control_msgs::srv::StepSimulation::Request>();
  step_req->steps = num_steps;
  auto step_future = step_client->async_send_request(step_req);
  ASSERT_EQ(step_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(step_future.get()->success);

  const double expected_time = time_after_pause + num_steps * sim_->model()->opt.timestep;
  EXPECT_NEAR(sim_->data()->time, expected_time, 1e-9) << "Time should advance by exactly " << num_steps << " steps";

  // Verify still paused after stepping
  const double time_after_step = sim_->data()->time;
  ASSERT_TRUE(wait_until([&]() { return sim_->data()->time != time_after_step; }, std::chrono::milliseconds(200)) ==
              false)
      << "Time should not advance after steps complete";

  // The simulation, confirm the sim restarts
  pause_req->paused = false;
  pause_future = pause_client->async_send_request(pause_req);
  ASSERT_EQ(pause_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(pause_future.get()->success);
  ASSERT_TRUE(wait_until([&]() { return sim_->data()->time > time_after_step; }))
      << "Time should advance after unpausing";
}

TEST_F(MujocoSimulationTest, ResetWorldTest)
{
  ASSERT_TRUE(initialize_sim());

  // Set a known initial state and capture it
  sim_->data()->qpos[0] = 0.0;
  sim_->data()->qvel[0] = 0.0;
  sim_->data()->ctrl[0] = 0.0;
  sim_->capture_initial_state();

  // Kick off the sim and wait for it to start
  sim_->start_physics_thread();
  ASSERT_TRUE(wait_until([this]() { return sim_->data()->time > 0.0; })) << "Simulation did not start stepping";

  // Setup service clients
  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto pause_client = node_->create_client<mujoco_ros2_control_msgs::srv::SetPause>(ns + "/set_pause");
  auto reset_client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(pause_client->wait_for_service(std::chrono::seconds(5))) << "set_pause service not found";
  ASSERT_TRUE(reset_client->wait_for_service(std::chrono::seconds(5))) << "reset_world service not found";

  // Let the simulation run so state drifts from initial
  ASSERT_TRUE(wait_until([this]() { return sim_->data()->time > 0.5; }));
  EXPECT_NE(sim_->data()->qpos[0], 0.0) << "Is the simulation not running?";

  // Record the clock time before reset
  const double time_before_reset = sim_->data()->time;
  EXPECT_GT(time_before_reset, 0.0);

  // Pause before resetting so we can inspect state without it changing
  auto pause_req = std::make_shared<mujoco_ros2_control_msgs::srv::SetPause::Request>();
  pause_req->paused = true;
  auto pause_future = pause_client->async_send_request(pause_req);
  ASSERT_EQ(pause_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(pause_future.get()->success);

  // Wait for pause to take effect
  ASSERT_TRUE(wait_for_pause()) << "Simulation did not pause";

  // Call reset_world with no keyframe (resets to captured initial state)
  auto reset_req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  auto reset_future = reset_client->async_send_request(reset_req);
  ASSERT_EQ(reset_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto reset_resp = reset_future.get();
  ASSERT_TRUE(reset_resp->success) << "reset_world failed: " << reset_resp->message;

  // Verify state was restored to initial conditions from the start
  EXPECT_NEAR(sim_->data()->qpos[0], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qvel[0], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->ctrl[0], 0.0, 1e-9);

  // Verify applied forces were cleared
  EXPECT_DOUBLE_EQ(sim_->data()->qfrc_applied[0], 0.0) << "qfrc_applied should be zero after reset";
  for (int i = 0; i < 6 * sim_->model()->nbody; ++i)
  {
    EXPECT_DOUBLE_EQ(sim_->data()->xfrc_applied[i], 0.0) << "xfrc_applied[" << i << "] should be zero after reset";
  }

  // Verify clock was NOT reset (continuity preserved)
  EXPECT_GE(sim_->data()->time, time_before_reset) << "Clock should not go backwards after reset";

  // Unpause and verify simulation continues from the reset state
  pause_req->paused = false;
  pause_future = pause_client->async_send_request(pause_req);
  ASSERT_EQ(pause_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(pause_future.get()->success);

  const double time_after_reset = sim_->data()->time;
  ASSERT_TRUE(wait_until([&]() { return sim_->data()->time > time_after_reset; }))
      << "Time should advance after unpausing post-reset";
}

TEST_F(MujocoSimulationTest, SetFreeJointStateSetsPoseAndVelocity)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5))) << "set_free_joint_state service not found";

  // Before the service call
  const int qpos_adr = 1;
  const int qvel_adr = 1;
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 0], 1.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 1], 0.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 2], 1.0);
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 3], 1.0, 1e-9);  // w
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 4], 0.0, 1e-9);  // x
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 5], 0.0, 1e-9);  // y
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 6], 0.0, 1e-9);  // z

  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr + 0], 0.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr + 5], 0.0);

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.pose.pose.position.x = 2.0;
  entry.pose.pose.position.y = 3.0;
  entry.pose.pose.position.z = 4.0;
  entry.pose.pose.orientation.w = std::sqrt(0.5);
  entry.pose.pose.orientation.x = std::sqrt(0.5);
  entry.pose.pose.orientation.y = 0.0;
  entry.pose.pose.orientation.z = 0.0;
  entry.twist.twist.linear.x = 0.1;
  entry.twist.twist.angular.z = 0.2;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 0], 2.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 1], 3.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 2], 4.0);
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 3], std::sqrt(0.5), 1e-9);  // w
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 4], std::sqrt(0.5), 1e-9);  // x
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 5], 0.0, 1e-9);             // y
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 6], 0.0, 1e-9);             // z

  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr + 0], 0.1);
  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr + 5], 0.2);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateDefaultsToZeroVelocity)
{
  ASSERT_TRUE(initialize_sim());

  // Give the free object a non-zero velocity before resetting.
  sim_->data()->qvel[1] = 5.0;

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.pose.pose.position.z = 2.0;
  // twist left at its default (all-zero) -- object should come to rest.

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(future.get()->success);

  for (int i = 1; i < 7; ++i)
  {
    EXPECT_DOUBLE_EQ(sim_->data()->qvel[i], 0.0) << "qvel[" << i << "] should be reset to zero";
  }
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRejectsUnknownBody)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "nonexistent_body";

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRejectsNonFreeBody)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "pendulum";  // driven by a hinge joint, not a free joint

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRelativeToBody)
{
  ASSERT_TRUE(initialize_sim());

  // Rotate "pendulum" 90 degrees about its hinge axis (0 1 0), so its world pose becomes
  // pos=(0, 0, 1), quat=(cos45, 0, sin45, 0). The hinge's own origin does not translate,
  // only rotates, so this exercises the rotation part of the pose composition.
  sim_->data()->qpos[0] = M_PI_2;
  mj_forward(sim_->model(), sim_->data());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.pose.header.frame_id = "pendulum";
  entry.pose.pose.position.x = 1.0;     // rel_pos, rotated 90 deg about Y: (1,0,0) -> (0,0,-1)
  entry.pose.pose.orientation.w = 1.0;  // identity relative orientation
  // twist.header.frame_id left empty (world frame), independent of the pose's reference frame.
  entry.twist.twist.linear.x = 5.0;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  const int qpos_adr = 1;
  const int qvel_adr = 1;
  // world_pos = ref_pos(0,0,1) + rotate_by_ref_quat(rel_pos(1,0,0)) = (0,0,1) + (0,0,-1) = (0,0,0)
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 0], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 1], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 2], 0.0, 1e-9);
  // world_quat = ref_quat * identity = ref_quat = (cos45, 0, sin45, 0)
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 3], std::sqrt(0.5), 1e-9);  // w
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 4], 0.0, 1e-9);             // x
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 5], std::sqrt(0.5), 1e-9);  // y
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 6], 0.0, 1e-9);             // z
  // twist stayed in the world frame, unaffected by the pose's reference frame.
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 0], 5.0, 1e-9);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRelativeToBodyRotatesTwist)
{
  ASSERT_TRUE(initialize_sim());

  // Rotate "pendulum" 90 degrees about its hinge axis (0 1 0), so its world pose becomes
  // pos=(0, 0, 1), quat=(cos45, 0, sin45, 0) similar setup as SetFreeJointStateRelativeToBody,
  // which rotates a reference-frame vector (1,0,0) to world (0,0,-1).
  sim_->data()->qpos[0] = M_PI_2;
  mj_forward(sim_->model(), sim_->data());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  // pose.header.frame_id is left empty (world frame) -- pose is not under test here, and this
  // demonstrates that twist's reference frame is resolved independently of pose's.
  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.twist.header.frame_id = "pendulum";
  entry.twist.twist.linear.x = 1.0;   // rotated 90 deg about Y: (1,0,0) -> (0,0,-1)
  entry.twist.twist.angular.x = 1.0;  // rotated 90 deg about Y: (1,0,0) -> (0,0,-1)

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  const int qvel_adr = 1;
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 0], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 1], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 2], -1.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 3], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 4], 0.0, 1e-9);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 5], -1.0, 1e-9);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRejectsUnknownPoseFrame)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  const int qpos_adr = 1;
  const double original_x = sim_->data()->qpos[qpos_adr + 0];

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.pose.header.frame_id = "nonexistent_body";
  entry.pose.pose.position.x = 5.0;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 0], original_x);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRejectsUnknownTwistFrame)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  const int qpos_adr = 1;
  const int qvel_adr = 1;
  const double original_x = sim_->data()->qpos[qpos_adr + 0];
  const double original_qvel_x = sim_->data()->qvel[qvel_adr + 0];

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.pose.pose.position.x = 5.0;  // valid, world-frame pose -- must not be applied either
  entry.twist.header.frame_id = "nonexistent_body";
  entry.twist.twist.linear.x = 1.0;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 0], original_x);
  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr + 0], original_qvel_x);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateSetsMultipleBodies)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  // Resolve addresses per body rather than hardcoding, since "free_object_2" sits after
  // "free_object" (qpos 8 / qvel 7) in the model's qpos/qvel layout.
  const int free_object_id = mj_name2id(sim_->model(), mjOBJ_BODY, "free_object");
  const int free_object_2_id = mj_name2id(sim_->model(), mjOBJ_BODY, "free_object_2");
  ASSERT_NE(free_object_id, -1);
  ASSERT_NE(free_object_2_id, -1);

  auto qpos_adr_for = [&](int body_id) {
    for (int i = 0; i < sim_->model()->njnt; ++i)
    {
      if (sim_->model()->jnt_bodyid[i] == body_id && sim_->model()->jnt_type[i] == mjJNT_FREE)
      {
        return sim_->model()->jnt_qposadr[i];
      }
    }
    return -1;
  };
  auto qvel_adr_for = [&](int body_id) {
    for (int i = 0; i < sim_->model()->njnt; ++i)
    {
      if (sim_->model()->jnt_bodyid[i] == body_id && sim_->model()->jnt_type[i] == mjJNT_FREE)
      {
        return sim_->model()->jnt_dofadr[i];
      }
    }
    return -1;
  };

  const int qpos_adr_1 = qpos_adr_for(free_object_id);
  const int qvel_adr_1 = qvel_adr_for(free_object_id);
  const int qpos_adr_2 = qpos_adr_for(free_object_2_id);
  const int qvel_adr_2 = qvel_adr_for(free_object_2_id);
  ASSERT_NE(qpos_adr_1, -1);
  ASSERT_NE(qpos_adr_2, -1);

  mujoco_ros2_control_msgs::msg::FreeJointState entry_1;
  entry_1.name = "free_object";
  entry_1.pose.pose.position.x = 2.0;
  entry_1.pose.pose.position.y = 3.0;
  entry_1.pose.pose.position.z = 4.0;
  entry_1.pose.pose.orientation.w = 1.0;
  entry_1.twist.twist.linear.x = 0.1;

  mujoco_ros2_control_msgs::msg::FreeJointState entry_2;
  entry_2.name = "free_object_2";
  entry_2.pose.pose.position.x = 5.0;
  entry_2.pose.pose.position.y = 6.0;
  entry_2.pose.pose.position.z = 7.0;
  entry_2.pose.pose.orientation.w = 1.0;
  entry_2.twist.twist.linear.y = 0.2;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry_1);
  req->free_joints.push_back(entry_2);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr_1 + 0], 2.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr_1 + 1], 3.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr_1 + 2], 4.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr_1 + 0], 0.1);

  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr_2 + 0], 5.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr_2 + 1], 6.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr_2 + 2], 7.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qvel[qvel_adr_2 + 1], 0.2);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRejectsBatchAtomically)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  const int qpos_adr = 1;
  const double original_x = sim_->data()->qpos[qpos_adr + 0];

  // One valid entry followed by one invalid entry (unknown body). Since the batch must be
  // applied atomically, the whole request should be rejected and the valid entry's target
  // must be left untouched.
  mujoco_ros2_control_msgs::msg::FreeJointState valid_entry;
  valid_entry.name = "free_object";
  valid_entry.pose.pose.position.x = 42.0;

  mujoco_ros2_control_msgs::msg::FreeJointState invalid_entry;
  invalid_entry.name = "nonexistent_body";

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(valid_entry);
  req->free_joints.push_back(invalid_entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[qpos_adr + 0], original_x)
      << "Valid entry must not be applied when another entry in the same batch is invalid";
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
