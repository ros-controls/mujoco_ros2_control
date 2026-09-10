// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
// A simple representative FTS suite
// Body orientation is identity so body-frame and world-frame forces coincide.
constexpr const char* kMjcf = R"(
<mujoco model="fts_grav_comp_test">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <!-- Main body with FT sensor site -->
    <body name="sensor_body" pos="0 0 1">
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <geom type="box" size="0.05 0.05 0.05" rgba="0.5 0.5 0.5 0.5"/>

      <!-- Site 1: Primary FT sensor location -->
      <site name="ft_sensor_site" pos="0.05 0 0"/>

      <!-- Site 2: Offset by 0.1m in Z direction, same orientation -->
      <site name="offset_site" pos="0.05 0 0.1"/>

      <!-- Site 3: Offset by 0.05m in X direction, and rotated 90 degrees
           around Y-axis the mass is now purely in the z dimension -->
      <site name="offset_rotated_site" pos="0.1 0 0" euler="0 90 0"/>

      <!-- Child body with 10kg mass attached at sensor site -->
      <body name="mass_body" pos="0.15 0 0">
        <inertial pos="0 0 0" mass="10" diaginertia="0.1 0.1 0.1"/>
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
