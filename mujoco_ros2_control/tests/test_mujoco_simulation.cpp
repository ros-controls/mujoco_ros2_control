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

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include <mujoco_ros2_control/mujoco_simulation.hpp>
#include <mujoco_ros2_control_msgs/msg/free_joint_state.hpp>
#include <mujoco_ros2_control_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_control_msgs/srv/set_free_joint_state.hpp>

namespace
{

// Basic model for executing unit tests: a hinge joint with an actuator, plus two free-floating
// bodies ("free_object", "free_object_2") for exercising the free-joint state service.
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

constexpr double TEST_TOLERANCE = 1e-9;

constexpr mjtNum kNaN = std::numeric_limits<mjtNum>::quiet_NaN();

/**
 * @brief Plugin implementing the current API, driving an injected callback on every update.
 *
 * The callback runs on the physics thread with the sim mutex held, so anything it records for the
 * test thread to inspect must be atomic.
 */
class TestControlPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr, const mjModel*, mjData*) override
  {
    return true;
  }

  void update(mjData* control_data) override
  {
    if (on_update)
    {
      on_update(control_data, get_sim_data(), get_mujoco_model());
    }
    update_count.fetch_add(1);
  }

  void cleanup() override
  {
  }

  std::function<void(mjData*, const mjData*, const mjModel*)> on_update;
  std::atomic<uint64_t> update_count{ 0 };
};

/**
 * @brief Plugin implementing *only* the deprecated two-argument update(), to pin the compatibility
 * contract: it must keep receiving live, readable mjData rather than the NaN command buffer.
 */
class LegacyTestPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr, const mjModel*, mjData*) override
  {
    return true;
  }

  void update(const mjModel* model, mjData* data) override
  {
    if (on_update)
    {
      on_update(model, data);
    }
    update_count.fetch_add(1);
  }

  void cleanup() override
  {
  }

  std::function<void(const mjModel*, mjData*)> on_update;
  std::atomic<uint64_t> update_count{ 0 };
};

/**
 * @brief Shared-ownership slot for values a plugin callback records for the test to assert on.
 *
 * The physics loop keeps calling registered plugins until the simulation is shut down in
 * TearDown, which is *after* the test body's locals have been destroyed. Capturing a stack local
 * by reference would therefore leave the callback writing through a dangling reference. Giving the
 * slot shared ownership keeps it alive for as long as the callback that writes it.
 */
template <typename T>
std::shared_ptr<std::atomic<T>> make_recorder(T initial)
{
  return std::make_shared<std::atomic<T>>(initial);
}

// True if any entry of the array is NaN. Used to prove the sentinel never leaks into the sim.
bool contains_nan(const mjtNum* values, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (std::isnan(values[i]))
    {
      return true;
    }
  }
  return false;
}

// True if every entry of the array is NaN, i.e. the buffer arrived fully unset.
bool all_nan(const mjtNum* values, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (!std::isnan(values[i]))
    {
      return false;
    }
  }
  return true;
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

  // Construct, initialize and register a plugin, mirroring what the hardware interface does in
  // load_mujoco_plugins(). Replaces any previously registered plugins.
  template <typename PluginT>
  std::shared_ptr<PluginT> add_plugin()
  {
    auto plugin = std::make_shared<PluginT>();
    EXPECT_TRUE(plugin->initialize(node_, sim_->model(), sim_->data()));
    sim_->register_plugins({ plugin });
    return plugin;
  }

  // Returns an all-NaN command vector sized for the model's actuators, i.e. "command nothing".
  std::vector<mjtNum> unset_actuator_commands() const
  {
    return std::vector<mjtNum>(sim_->model()->nu, kNaN);
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

  // Seed a known control value so we can prove an unset (NaN) command leaves it alone.
  sim_->data()->ctrl[0] = 0.25;

  // Staging an all-NaN command means "command nothing", so ctrl must survive untouched. This is
  // what lets the hardware interface skip passive / uncommanded actuators without first mirroring
  // the simulation's current ctrl into the buffer.
  auto commands = unset_actuator_commands();
  sim_->stage_actuator_commands(commands);

  sim_->start_physics_thread();
  const uint64_t start_steps = sim_->step_count();
  ASSERT_TRUE(wait_until([&]() { return sim_->step_count() > start_steps + 5; }));
  EXPECT_DOUBLE_EQ(sim_->data()->ctrl[0], 0.25) << "An unset (NaN) command overwrote the existing ctrl value";

  // A finite command is merged into the sim data by the physics loop before the next step.
  commands[0] = 0.75;
  sim_->stage_actuator_commands(commands);
  EXPECT_TRUE(wait_until([this]() { return sim_->data()->ctrl[0] == 0.75; }))
      << "Staged actuator command was not applied by the physics loop";
}

TEST_F(MujocoSimulationTest, XfrcAppliedTests)
{
  ASSERT_TRUE(initialize_sim());

  // A plugin commands a Cartesian force by writing it into control_data; the physics loop merges
  // it into the sim data immediately before stepping.
  constexpr int body_id = 1;
  auto plugin = add_plugin<TestControlPlugin>();
  plugin->on_update = [](mjData* control_data, const mjData*, const mjModel*) {
    control_data->xfrc_applied[body_id * 6 + 0] = 1.0;
    control_data->xfrc_applied[body_id * 6 + 1] = 2.0;
    control_data->xfrc_applied[body_id * 6 + 2] = 3.0;
  };

  sim_->start_physics_thread();
  EXPECT_TRUE(wait_until([this]() {
    return sim_->data()->xfrc_applied[body_id * 6 + 0] == 1.0 && sim_->data()->xfrc_applied[body_id * 6 + 1] == 2.0 &&
           sim_->data()->xfrc_applied[body_id * 6 + 2] == 3.0;
  })) << "Plugin Cartesian force never reached the simulation data";
}

TEST_F(MujocoSimulationTest, PluginControlDataIsNanFilled)
{
  ASSERT_TRUE(initialize_sim());

  auto arrived_unset = make_recorder<bool>(true);
  auto plugin = add_plugin<TestControlPlugin>();
  plugin->on_update = [arrived_unset](mjData* control_data, const mjData*, const mjModel* model) {
    // Every commandable field must arrive fully unset, so nothing a plugin wrote on a previous
    // iteration is silently re-applied on this one.
    const bool unset = all_nan(control_data->ctrl, model->nu) && all_nan(control_data->qfrc_applied, model->nv) &&
                       all_nan(control_data->xfrc_applied, 6 * model->nbody) &&
                       all_nan(control_data->qpos, model->nq) && all_nan(control_data->qvel, model->nv);
    if (!unset)
    {
      arrived_unset->store(false);
    }
    // Write something, so the next iteration proves the buffer really is refilled.
    control_data->ctrl[0] = 0.5;
  };

  sim_->start_physics_thread();
  ASSERT_TRUE(wait_until([&plugin]() { return plugin->update_count.load() >= 3; }))
      << "Plugin update was never called from the physics loop";
  EXPECT_TRUE(arrived_unset->load()) << "control_data was not NaN-filled before a plugin update";
}

TEST_F(MujocoSimulationTest, PluginPartialWriteLeavesRestUntouched)
{
  ASSERT_TRUE(initialize_sim());

  auto plugin = add_plugin<TestControlPlugin>();
  plugin->on_update = [](mjData* control_data, const mjData*, const mjModel*) {
    // Command a single actuator and nothing else.
    control_data->ctrl[0] = 0.4;
  };

  sim_->start_physics_thread();
  ASSERT_TRUE(wait_until([&plugin]() { return plugin->update_count.load() >= 3; }));

  const mjModel* model = sim_->model();
  const mjData* data = sim_->data();

  // Fields the plugin left unset must never receive the NaN sentinel itself.
  EXPECT_FALSE(contains_nan(data->ctrl, model->nu));
  EXPECT_FALSE(contains_nan(data->qpos, model->nq));
  EXPECT_FALSE(contains_nan(data->qvel, model->nv));
  EXPECT_FALSE(contains_nan(data->qfrc_applied, model->nv));
  EXPECT_FALSE(contains_nan(data->xfrc_applied, 6 * model->nbody));

  // Nothing writes qfrc_applied in this test, so it must remain at its reset value.
  for (int i = 0; i < model->nv; ++i)
  {
    EXPECT_DOUBLE_EQ(data->qfrc_applied[i], 0.0) << "qfrc_applied[" << i << "] was disturbed";
  }
}

TEST_F(MujocoSimulationTest, PluginCanCommandQposAndQvel)
{
  ASSERT_TRUE(initialize_sim());

  // Pin the hinge by commanding its position and velocity every iteration. The merge happens
  // immediately before the step, so the joint should stay within one timestep of the command.
  constexpr mjtNum kPinnedPosition = 0.4;
  auto plugin = add_plugin<TestControlPlugin>();
  plugin->on_update = [](mjData* control_data, const mjData*, const mjModel*) {
    control_data->qpos[0] = kPinnedPosition;
    control_data->qvel[0] = 0.0;
  };

  sim_->start_physics_thread();
  ASSERT_TRUE(wait_until([&plugin]() { return plugin->update_count.load() >= 5; }));
  EXPECT_NEAR(sim_->data()->qpos[0], kPinnedPosition, 1e-2) << "Plugin qpos command did not reach the simulation";
}

TEST_F(MujocoSimulationTest, PluginSeesLiveSimState)
{
  ASSERT_TRUE(initialize_sim());

  auto observed_time = make_recorder<double>(-1.0);
  auto saw_valid_state = make_recorder<bool>(false);
  auto plugin = add_plugin<TestControlPlugin>();
  plugin->on_update = [observed_time, saw_valid_state](mjData*, const mjData* sim_data, const mjModel* model) {
    if (sim_data != nullptr && model != nullptr && !contains_nan(sim_data->qpos, model->nq))
    {
      saw_valid_state->store(true);
      observed_time->store(sim_data->time);
    }
  };

  sim_->start_physics_thread();
  // The observed sim time must advance, proving get_sim_data() tracks the live data rather than a
  // stale copy captured at init.
  EXPECT_TRUE(wait_until([observed_time]() { return observed_time->load() > 0.0; }))
      << "get_sim_data() never reported advancing simulation time";
  EXPECT_TRUE(saw_valid_state->load()) << "get_sim_data() did not expose readable state";
}

TEST_F(MujocoSimulationTest, LegacyPluginReceivesLiveData)
{
  ASSERT_TRUE(initialize_sim());

  // A plugin that only implements the deprecated two-argument update() must still be handed live,
  // readable mjData -- not the NaN command buffer -- and its direct writes must take effect.
  constexpr int body_id = 2;
  auto saw_readable_data = make_recorder<bool>(false);
  auto observed_time = make_recorder<double>(-1.0);
  auto plugin = add_plugin<LegacyTestPlugin>();
  plugin->on_update = [saw_readable_data, observed_time](const mjModel* model, mjData* data) {
    if (model != nullptr && data != nullptr && !contains_nan(data->qpos, model->nq) && !std::isnan(data->time))
    {
      saw_readable_data->store(true);
      observed_time->store(data->time);
    }
    data->xfrc_applied[body_id * 6 + 0] = 5.0;
  };

  sim_->start_physics_thread();
  ASSERT_TRUE(wait_until([&plugin]() { return plugin->update_count.load() >= 3; }))
      << "Deprecated update() overload was never called";
  EXPECT_TRUE(saw_readable_data->load()) << "Legacy plugin received a NaN buffer instead of live sim data";
  EXPECT_TRUE(wait_until([observed_time]() { return observed_time->load() > 0.0; }));
  EXPECT_DOUBLE_EQ(sim_->data()->xfrc_applied[body_id * 6 + 0], 5.0)
      << "Legacy plugin's direct write to the live data did not survive the merge";
}

TEST_F(MujocoSimulationTest, ExistingXfrcSurvivesPluginMerge)
{
  ASSERT_TRUE(initialize_sim());

  // Forces already present in the sim data (e.g. the viewer's interactive drag) must survive on
  // bodies the plugin does not command, because the merge no longer overwrites the whole array.
  constexpr int untouched_body = 2;
  constexpr int commanded_body = 3;
  sim_->data()->xfrc_applied[untouched_body * 6 + 0] = 7.0;

  auto plugin = add_plugin<TestControlPlugin>();
  plugin->on_update = [](mjData* control_data, const mjData*, const mjModel*) {
    control_data->xfrc_applied[commanded_body * 6 + 0] = 9.0;
  };

  sim_->start_physics_thread();
  ASSERT_TRUE(wait_until([this]() { return sim_->data()->xfrc_applied[commanded_body * 6 + 0] == 9.0; }))
      << "Plugin force never reached the commanded body";
  EXPECT_DOUBLE_EQ(sim_->data()->xfrc_applied[untouched_body * 6 + 0], 7.0)
      << "Pre-existing force on an uncommanded body was clobbered by the merge";
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
  EXPECT_NEAR(sim_->data()->time, expected_time, TEST_TOLERANCE)
      << "Time should advance by exactly " << num_steps << " steps";

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
  EXPECT_NEAR(sim_->data()->qpos[0], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qvel[0], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->ctrl[0], 0.0, TEST_TOLERANCE);

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

TEST_F(MujocoSimulationTest, ResetWorldJointStateOverrides)
{
  ASSERT_TRUE(initialize_sim());

  // Capture a known initial state, then drift away from it.
  sim_->data()->qpos[0] = 0.0;
  sim_->data()->qvel[0] = 0.0;
  sim_->capture_initial_state();
  sim_->data()->qpos[0] = 0.3;
  sim_->data()->qvel[0] = 2.0;

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->state_overrides.joint_states.name = { "hinge" };
  req->state_overrides.joint_states.position = { 0.7 };
  req->state_overrides.joint_states.velocity = { 0.25 };

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  EXPECT_DOUBLE_EQ(sim_->data()->qpos[0], 0.7) << "Override should win over the restored initial position";
  EXPECT_DOUBLE_EQ(sim_->data()->qvel[0], 0.25) << "Override should win over the restored initial velocity";
}

TEST_F(MujocoSimulationTest, ResetWorldKeyframeAndOverrides)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState free_override;
  free_override.name = "free_object_2";
  free_override.pose.pose.position.x = 5.0;
  free_override.pose.pose.position.y = 6.0;
  free_override.pose.pose.position.z = 7.0;
  free_override.pose.pose.orientation.w = 1.0;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->keyframe = "home";  // hinge=0.5, free_object at (1,0,1), free_object_2 at (2,0,1)
  req->state_overrides.joint_states.name = { "hinge" };
  req->state_overrides.joint_states.position = { 0.25 };
  req->state_overrides.free_joints.push_back(free_override);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  // Overridden entries take their override values, everything else keeps the keyframe values.
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[0], 0.25);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[1], 1.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[2], 0.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[3], 1.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[8], 5.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[9], 6.0);
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[10], 7.0);
  // Unset twist in the free-joint override comes to rest, like set_free_joint_state.
  for (int i = 7; i < 13; ++i)
  {
    EXPECT_DOUBLE_EQ(sim_->data()->qvel[i], 0.0) << "qvel[" << i << "] should be zero";
  }
}

TEST_F(MujocoSimulationTest, ResetWorldOverridesResolveFramesAfterReset)
{
  ASSERT_TRUE(initialize_sim());

  // Move "free_object" away from its keyframe pose so pre-reset and post-reset frame
  // resolution give different answers.
  sim_->data()->qpos[1] = 10.0;
  mj_forward(sim_->model(), sim_->data());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState free_override;
  free_override.name = "free_object_2";
  free_override.pose.header.frame_id = "free_object";
  free_override.pose.pose.position.x = 0.5;
  free_override.pose.pose.orientation.w = 1.0;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->keyframe = "home";  // puts free_object at (1,0,1)
  req->state_overrides.free_joints.push_back(free_override);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  // Resolved against free_object's post-reset pose (1,0,1), not its pre-reset pose (10,0,1).
  EXPECT_NEAR(sim_->data()->qpos[8], 1.5, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qpos[9], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qpos[10], 1.0, TEST_TOLERANCE);
}

TEST_F(MujocoSimulationTest, ResetWorldInvalidOverrides)
{
  ASSERT_TRUE(initialize_sim());

  sim_->data()->qpos[0] = 0.0;
  sim_->capture_initial_state();
  sim_->data()->qpos[0] = 0.3;

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->state_overrides.joint_states.name = { "hinge", "nonexistent_joint" };
  req->state_overrides.joint_states.position = { 0.7, 0.1 };

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
  EXPECT_DOUBLE_EQ(sim_->data()->qpos[0], 0.3) << "An invalid override must leave the world un-reset";
}

TEST_F(MujocoSimulationTest, ResetWorldMalformedOverrides)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  // Mismatched position length.
  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->state_overrides.joint_states.name = { "hinge" };
  req->state_overrides.joint_states.position = { 0.1, 0.2 };
  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());

  // Effort overrides are not supported.
  req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->state_overrides.joint_states.name = { "hinge" };
  req->state_overrides.joint_states.effort = { 1.0 };
  future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());
}

TEST_F(MujocoSimulationTest, ResetWorldFreeJointInJointState)
{
  ASSERT_TRUE(initialize_sim());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>(ns + "/reset_world");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>();
  req->state_overrides.joint_states.name = { "free_object_joint" };
  req->state_overrides.joint_states.position = { 1.0 };

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  EXPECT_FALSE(resp->success);
  EXPECT_NE(resp->message.find("state_overrides.free_joints"), std::string::npos)
      << "Message should point free joints at the state_overrides.free_joints field, got: " << resp->message;
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
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 3], 1.0, TEST_TOLERANCE);  // w
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 4], 0.0, TEST_TOLERANCE);  // x
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 5], 0.0, TEST_TOLERANCE);  // y
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 6], 0.0, TEST_TOLERANCE);  // z

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
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 3], std::sqrt(0.5), TEST_TOLERANCE);  // w
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 4], std::sqrt(0.5), TEST_TOLERANCE);  // x
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 5], 0.0, TEST_TOLERANCE);             // y
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 6], 0.0, TEST_TOLERANCE);             // z

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

  // Rotate "pendulum" 90 degrees about Y, so its world pose becomes pos=(0,0,1),
  // quat=(cos45,0,sin45,0).
  sim_->data()->qpos[0] = M_PI_2;
  mj_forward(sim_->model(), sim_->data());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

  mujoco_ros2_control_msgs::msg::FreeJointState entry;
  entry.name = "free_object";
  entry.pose.header.frame_id = "pendulum";
  entry.pose.pose.position.x = 1.0;  // rotated 90 deg about Y: (1,0,0) -> (0,0,-1)
  entry.pose.pose.orientation.w = 1.0;
  // twist.header.frame_id left empty: world frame, independent of pose's reference frame.
  entry.twist.twist.linear.x = 5.0;

  auto req = std::make_shared<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request>();
  req->free_joints.push_back(entry);

  auto future = client->async_send_request(req);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_TRUE(resp->success) << resp->message;

  const int qpos_adr = 1;
  const int qvel_adr = 1;
  // world_pos = ref_pos(0,0,1) + rotate(rel_pos(1,0,0)) = (0,0,0)
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 0], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 1], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 2], 0.0, TEST_TOLERANCE);
  // world_quat = ref_quat * identity = ref_quat = (cos45, 0, sin45, 0)
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 3], std::sqrt(0.5), TEST_TOLERANCE);  // w
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 4], 0.0, TEST_TOLERANCE);             // x
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 5], std::sqrt(0.5), TEST_TOLERANCE);  // y
  EXPECT_NEAR(sim_->data()->qpos[qpos_adr + 6], 0.0, TEST_TOLERANCE);             // z
  // twist stayed in the world frame.
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 0], 5.0, TEST_TOLERANCE);
}

TEST_F(MujocoSimulationTest, SetFreeJointStateRelativeToBodyRotatesTwist)
{
  ASSERT_TRUE(initialize_sim());

  // Same rotated "pendulum" setup as SetFreeJointStateRelativeToBody, but here it is the
  // twist's reference frame -- pose.header.frame_id is left empty (world) to show they
  // resolve independently.
  sim_->data()->qpos[0] = M_PI_2;
  mj_forward(sim_->model(), sim_->data());

  const std::string ns = std::string(node_->get_fully_qualified_name());
  auto client = node_->create_client<mujoco_ros2_control_msgs::srv::SetFreeJointState>(ns + "/set_free_joint_state");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(5)));

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
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 0], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 1], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 2], -1.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 3], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 4], 0.0, TEST_TOLERANCE);
  EXPECT_NEAR(sim_->data()->qvel[qvel_adr + 5], -1.0, TEST_TOLERANCE);
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

  // The invalid second entry must reject the whole batch, leaving the first untouched.
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
