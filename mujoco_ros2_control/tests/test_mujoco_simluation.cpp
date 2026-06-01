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

namespace
{

// Basic model for executing unit tests, has a hinge joint and an
// actuator. Most importantly:
//    nu=1, nv=1, nq=1, nbody=2 (world + pendulum)
constexpr const char* kTestModel = R"(<?xml version="1.0"?>
<mujoco model="test_simulation">
  <option timestep="0.002"/>

  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <position name="hinge_pos" joint="hinge" kp="10"/>
  </actuator>

  <keyframe>
    <key name="home" qpos="0.5"/>
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

  EXPECT_EQ(sim_->model()->nq, 1);
  EXPECT_EQ(sim_->model()->nv, 1);
  EXPECT_EQ(sim_->model()->nu, 1);
  EXPECT_EQ(sim_->model()->nbody, 2);
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

  // Apply to sim and verify
  sim_->apply_control_data(control);
  EXPECT_DOUBLE_EQ(sim_->data()->ctrl[0], 0.75);
  EXPECT_DOUBLE_EQ(sim_->data()->qfrc_applied[0], 1.5);
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

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
