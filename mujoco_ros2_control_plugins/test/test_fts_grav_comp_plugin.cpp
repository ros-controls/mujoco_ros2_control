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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "fts_grav_comp_plugin.hpp"

namespace
{
// A simple representative FTS testing suite
// Note here that the FTS is attached to the child body, which agrees with
// the standard that the FTS is attached to the child body
constexpr const char* kMjcf = R"(
<mujoco model="fts_grav_comp_test">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <!-- Main body with FT sensor site -->
    <body name="sensor_body" pos="0 0 1">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <geom type="box" size="0.05 0.05 0.05" rgba="0.5 0.5 0.5 0.5"/>

      <!-- Child body with 10kg mass attached -->
      <body name="mass_body" pos="0 0 0">
        <!-- Site 1: Primary FT sensor location -->
        <site name="ft_sensor_site" pos="0.05 0 0"/>

        <!-- Site 2: Offset by 0.1m in Y direction, same orientation -->
        <site name="offset_site" pos="0.05 0.1 0.0"/>

        <!-- Site 3: Offset by 0.05m in X direction, and rotated 90 degrees
            around Y-axis the mass is now purely in the z dimension -->
        <site name="offset_rotated_site" pos="0.1 0 0" euler="0 90 0"/>

        <inertial pos="0.15 0 0" mass="10" diaginertia="0.1 0.1 0.1"/>
        <geom type="sphere" size="0.05" pos="0 0 0" rgba="1 0 0 0.2"/>
      </body>
    </body>
  </worldbody>

  <sensor>
    <force name="fts_sensor_force" site="ft_sensor_site"/>
    <torque name="fts_sensor_torque" site="ft_sensor_site"/>
    <!-- second fts located at the offset rotated site -->
    <force name="or_fts_sensor_force" site="offset_rotated_site"/>
    <torque name="or_fts_sensor_torque" site="offset_rotated_site"/>
  </sensor>
</mujoco>
)";
}  // namespace

class FtsGravCompPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("fts_grav_comp_test_node");
    plugin_node_ = node_->create_sub_node("fts_grav_comp_plugin");

    // Use two executor threads so a blocking service callback (sleeping for
    // its wrench duration) does not prevent other callbacks (e.g. subscription
    // delivery) from being dispatched.
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 2);
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
    char error[1024] = { 0 };
    mjSpec* spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;

    model_ = mj_compile(spec, nullptr);
    if (model_ == nullptr)
    {
      const char* ce = mjs_getError(spec);
      mj_deleteSpec(spec);
      FAIL() << (ce ? ce : "mj_compile failed");
    }
    mj_deleteSpec(spec);

    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
    mj_forward(model_, data_);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    executor_.reset();
    plugin_node_.reset();
    node_.reset();
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
  }

  /// Pre-declares and sets a parameter on plugin_node_ before init(), matching how launch
  /// parameter files set plugin parameters ahead of load_mujoco_plugins().
  template <typename T>
  void setParam(const std::string& name, const T& value)
  {
    const std::string resolved_name = "mujoco_plugins." + plugin_node_->get_sub_namespace() + "." + name;
    if (!plugin_node_->has_parameter(resolved_name))
    {
      plugin_node_->declare_parameter(resolved_name, rclcpp::ParameterValue(value));
    }
    plugin_node_->set_parameter(rclcpp::Parameter(resolved_name, value));
  }

  /// sets parameters for the plugin
  /// fts_grav_comp_plugin:
  ///   fts_sensor:
  ///     # frame the CoG is represented in
  ///     frame_id: ft_sensor_site
  ///     # specifies the center of gravity of the end effector
  ///     CoG:
  ///       pos:
  ///         - 0.1 # x
  ///         - 0.0 # y
  ///         - 0.0 # z
  ///       mass: 10.0 # mass
  ///   or_fts_sensor:
  ///     # frame the CoG is represented in
  ///     frame_id: offset_rotated_site
  ///     # specifies the center of gravity of the end effector
  ///     CoG:
  ///       pos:
  ///         - 0.0 # x
  ///         - 0.0 # y
  ///         - 0.05 # z
  ///       mass: 10.0 # mass
  void set_plugin_params()
  {
    setParam("fts_sensor.frame_id", std::string("ft_sensor_site"));
    setParam("fts_sensor.CoG.pos", std::vector<double>{ 0.1, 0.0, 0.0 });
    setParam("fts_sensor.CoG.mass", double{ 10.0 });

    setParam("or_fts_sensor.frame_id", std::string("offset_rotated_site"));
    setParam("or_fts_sensor.CoG.pos", std::vector<double>{ 0.0, 0.0, 0.05 });
    setParam("or_fts_sensor.CoG.mass", double{ 10.0 });
  }

  std::vector<double> get_ft_data(mujoco_ros2_control_plugins::FtsData fts)
  {
    std::vector<double> ft_data(6, 0);
    const mjtNum* sensordata_force = data_->sensordata + fts.sensor_adr_force;
    const mjtNum* sensordata_torque = data_->sensordata + fts.sensor_adr_torque;
    // everything must be negated to handle the way force and torque sensors are set up
    ft_data[0] = -sensordata_force[0];   // force x
    ft_data[1] = -sensordata_force[1];   // force y
    ft_data[2] = -sensordata_force[2];   // force z
    ft_data[3] = -sensordata_torque[0];  // torque x
    ft_data[4] = -sensordata_torque[1];  // torque y
    ft_data[5] = -sensordata_torque[2];  // torque z

    return ft_data;
  }

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

private:
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// can properly load the MJCF and initialize the plugin
TEST_F(FtsGravCompPluginTest, InitSucceeds)
{
  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

// reads in the plugin and verifies that the sensors loaded were correct
TEST_F(FtsGravCompPluginTest, SensorsLoaded)
{
  set_plugin_params();
  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  ASSERT_EQ(plugin.get_fts_data().size(), 2) << "Two sensors should have been loaded";
  EXPECT_EQ(plugin.get_fts_data()[0].sensor_name, "fts_sensor");
  EXPECT_EQ(plugin.get_fts_data()[1].sensor_name, "or_fts_sensor");
  plugin.cleanup();
}

// verifies that initialization fails if a site id is wrong
TEST_F(FtsGravCompPluginTest, FailOnNonExistentSite)
{
  set_plugin_params();
  setParam("fts_sensor.frame_id", std::string("non_existent_site"));
  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  ASSERT_FALSE(plugin.init(plugin_node_, model_, data_));

  plugin.cleanup();
}

// verifies that initialization fails if the name of the sensor is wrong
TEST_F(FtsGravCompPluginTest, FailOnNonExistentSensor)
{
  set_plugin_params();
  setParam("nonexistent_fts_sensor.frame_id", std::string("ft_sensor_site"));
  setParam("nonexistent_fts_sensor.CoG.pos", std::vector<double>{ 0.1, 0.0, 0.0 });
  setParam("nonexistent_fts_sensor.CoG.mass", double{ 10.0 });
  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  ASSERT_FALSE(plugin.init(plugin_node_, model_, data_));

  plugin.cleanup();
}

// verifies that the final wrench data is 0 after gravity compensation
TEST_F(FtsGravCompPluginTest, GravityIsComped)
{
  set_plugin_params();

  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.update(model_, data_);

  // expected that the FT data should be 0 after gravity compensation
  std::vector<double> expected_data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // check the first FT sensor
  std::vector<double> real_data_0 = get_ft_data(plugin.get_fts_data()[0]);
  EXPECT_NEAR(real_data_0[0], expected_data[0], 1e-9);
  EXPECT_NEAR(real_data_0[1], expected_data[1], 1e-9);
  EXPECT_NEAR(real_data_0[2], expected_data[2], 1e-9);
  EXPECT_NEAR(real_data_0[3], expected_data[3], 1e-9);
  EXPECT_NEAR(real_data_0[4], expected_data[4], 1e-9);
  EXPECT_NEAR(real_data_0[5], expected_data[5], 1e-9);

  // check the second FT sensor
  std::vector<double> real_data_1 = get_ft_data(plugin.get_fts_data()[1]);
  EXPECT_NEAR(real_data_1[0], expected_data[0], 1e-9);
  EXPECT_NEAR(real_data_1[1], expected_data[1], 1e-9);
  EXPECT_NEAR(real_data_1[2], expected_data[2], 1e-9);
  EXPECT_NEAR(real_data_1[3], expected_data[3], 1e-9);
  EXPECT_NEAR(real_data_1[4], expected_data[4], 1e-9);
  EXPECT_NEAR(real_data_1[5], expected_data[5], 1e-9);

  plugin.cleanup();
}

// verifies that the the same data sees the real weight after gravity compensation mass is 0
TEST_F(FtsGravCompPluginTest, GravityIsNotComped)
{
  set_plugin_params();
  setParam("fts_sensor.CoG.mass", double{ 0.0 });
  setParam("or_fts_sensor.CoG.mass", double{ 0.0 });
  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.update(model_, data_);

  // force FTS 0 (negative z direction)
  // mass * gravity
  double force_0 = 10.0 * -9.81;
  // torque FTS 1 (positive y direction)
  // mass * gravity * distance
  double torque_0 = 10.0 * 9.81 * 0.1;

  // check the first FT sensor
  std::vector<double> real_data_0 = get_ft_data(plugin.get_fts_data()[0]);
  EXPECT_NEAR(real_data_0[0], 0.0, 1e-9);
  EXPECT_NEAR(real_data_0[1], 0.0, 1e-9);
  EXPECT_NEAR(real_data_0[2], force_0, 1e-9);
  EXPECT_NEAR(real_data_0[3], 0.0, 1e-9);
  EXPECT_NEAR(real_data_0[4], torque_0, 1e-9);
  EXPECT_NEAR(real_data_0[5], 0.0, 1e-9);

  // force FTS 0 (positive x direction)
  // mass * gravity
  double force_1 = 10.0 * 9.81;
  // torque FTS 1 (positive y direction)
  // mass * gravity * distance
  double torque_1 = 10.0 * 9.81 * 0.05;

  // check the second FT sensor
  std::vector<double> real_data_1 = get_ft_data(plugin.get_fts_data()[1]);
  EXPECT_NEAR(real_data_1[0], force_1, 1e-9);
  EXPECT_NEAR(real_data_1[1], 0.0, 1e-9);
  EXPECT_NEAR(real_data_1[2], 0.0, 1e-9);
  EXPECT_NEAR(real_data_1[3], 0.0, 1e-9);
  EXPECT_NEAR(real_data_1[4], torque_1, 1e-9);
  EXPECT_NEAR(real_data_1[5], 0.0, 1e-9);

  plugin.cleanup();
}

// verifies that the final wrench data is 0 after gravity compensation
TEST_F(FtsGravCompPluginTest, GravityIsCompedWithFrameIdDifferentFromSensorId)
{
  set_plugin_params();
  setParam("fts_sensor.frame_id", std::string("offset_site"));
  setParam("fts_sensor.CoG.pos", std::vector<double>{ 0.1, -0.1, 0.0 });
  setParam("or_fts_sensor.frame_id", std::string("offset_site"));
  setParam("or_fts_sensor.CoG.pos", std::vector<double>{ 0.1, -0.1, 0.0 });

  mujoco_ros2_control_plugins::FtsGravCompPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.update(model_, data_);

  // expected that the FT data should be 0 after gravity compensation
  std::vector<double> expected_data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  // check the first FT sensor
  std::vector<double> real_data_0 = get_ft_data(plugin.get_fts_data()[0]);
  EXPECT_NEAR(real_data_0[0], expected_data[0], 1e-9);
  EXPECT_NEAR(real_data_0[1], expected_data[1], 1e-9);
  EXPECT_NEAR(real_data_0[2], expected_data[2], 1e-9);
  EXPECT_NEAR(real_data_0[3], expected_data[3], 1e-9);
  EXPECT_NEAR(real_data_0[4], expected_data[4], 1e-9);
  EXPECT_NEAR(real_data_0[5], expected_data[5], 1e-9);

  // check the second FT sensor
  std::vector<double> real_data_1 = get_ft_data(plugin.get_fts_data()[1]);
  EXPECT_NEAR(real_data_1[0], expected_data[0], 1e-9);
  EXPECT_NEAR(real_data_1[1], expected_data[1], 1e-9);
  EXPECT_NEAR(real_data_1[2], expected_data[2], 1e-9);
  EXPECT_NEAR(real_data_1[3], expected_data[3], 1e-9);
  EXPECT_NEAR(real_data_1[4], expected_data[4], 1e-9);
  EXPECT_NEAR(real_data_1[5], expected_data[5], 1e-9);

  plugin.cleanup();
}
