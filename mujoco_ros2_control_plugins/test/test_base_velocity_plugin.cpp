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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "base_velocity_plugin.hpp"

namespace
{
// A free-floating sphere (nv = 6: 3 translational + 3 rotational DOFs), no gravity so the
// body only moves under the servo forces the plugin applies. Identity orientation means
// body-frame and world-frame vectors coincide at t=0, which keeps the arithmetic in the
// tests simple to reason about.
constexpr const char* kMjcf = R"(
<mujoco model="base_velocity_test">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base_link" pos="0 0 1">
      <freejoint/>
      <inertial mass="1.0" pos="0 0 0" diaginertia="0.1 0.1 0.1"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";
}  // namespace

class BaseVelocityPluginTest : public ::testing::Test
{
protected:
  using Twist = geometry_msgs::msg::Twist;

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
    node_ = std::make_shared<rclcpp::Node>("base_velocity_test_node");
    plugin_node_ = node_->create_sub_node("base_velocity_plugin");

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

  /// Declares "body" (defaulting to "base_link") and initializes a plugin instance with it.
  std::unique_ptr<mujoco_ros2_control_plugins::BaseVelocityPlugin>
  makeInitializedPlugin(const std::string& body_name = "base_link")
  {
    if (!plugin_node_->has_parameter("body"))
    {
      plugin_node_->declare_parameter("body", body_name);
    }
    auto plugin = std::make_unique<mujoco_ros2_control_plugins::BaseVelocityPlugin>();
    EXPECT_TRUE(plugin->init(plugin_node_, model_, data_));
    return plugin;
  }

  /// Publishes a Twist on the plugin's cmd_vel topic and waits briefly for delivery.
  void publishTwist(const std::string& topic, double vx, double vy, double wz)
  {
    auto pub = plugin_node_->create_publisher<Twist>(topic, rclcpp::SystemDefaultsQoS());
    Twist msg;
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.angular.z = wz;
    // Give the subscription time to match before publishing (intra-process/local discovery).
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  }

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

private:
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(BaseVelocityPluginTest, InitFailsForUnknownBody)
{
  plugin_node_->declare_parameter("body", std::string("does_not_exist"));

  mujoco_ros2_control_plugins::BaseVelocityPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

TEST_F(BaseVelocityPluginTest, InitSucceedsForFreeJointBody)
{
  auto plugin = makeInitializedPlugin();
  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, ZeroCommandProducesNoForce)
{
  auto plugin = makeInitializedPlugin();

  plugin->update(model_, data_);

  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    EXPECT_DOUBLE_EQ(data_->xfrc_applied[i], 0.0);
  }

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, CommandAppliesForceTowardTarget)
{
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/1.0, /*vy=*/0.0, /*wz=*/0.0);

  plugin->update(model_, data_);

  const int base_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
  ASSERT_GE(base_id, 0);

  // Body starts at rest, commanded +X velocity => positive world-X force at identity
  // orientation, and no spurious force/torque on the other axes.
  EXPECT_GT(data_->xfrc_applied[base_id * 6 + 0], 0.0);
  EXPECT_NEAR(data_->xfrc_applied[base_id * 6 + 1], 0.0, 1e-9);
  EXPECT_NEAR(data_->xfrc_applied[base_id * 6 + 2], 0.0, 1e-9);
  EXPECT_NEAR(data_->xfrc_applied[base_id * 6 + 3], 0.0, 1e-9);
  EXPECT_NEAR(data_->xfrc_applied[base_id * 6 + 4], 0.0, 1e-9);
  EXPECT_NEAR(data_->xfrc_applied[base_id * 6 + 5], 0.0, 1e-9);

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, ServoConvergesLinearVelocityToCommand)
{
  plugin_node_->declare_parameter("kv_linear", 50.0);
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/0.5, /*vy=*/0.0, /*wz=*/0.0);

  const int base_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
  ASSERT_GE(base_id, 0);

  // Drive the servo + physics forward for a few hundred steps and check that the
  // measured body-frame x velocity converges to the 0.5 m/s command.
  for (int i = 0; i < 500; ++i)
  {
    plugin->update(model_, data_);
    mj_step(model_, data_);
  }

  mjtNum vel6[6];
  mj_objectVelocity(model_, data_, mjOBJ_BODY, base_id, vel6, /*flg_local=*/1);
  // vel6 layout is (rot:lin): [wx,wy,wz, vx,vy,vz]
  EXPECT_NEAR(vel6[3], 0.5, 0.05);
  EXPECT_NEAR(vel6[4], 0.0, 0.05);
}

TEST_F(BaseVelocityPluginTest, ServoConvergesYawRateToCommand)
{
  plugin_node_->declare_parameter("kv_yaw", 10.0);
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/0.0, /*vy=*/0.0, /*wz=*/1.0);

  const int base_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
  ASSERT_GE(base_id, 0);

  for (int i = 0; i < 500; ++i)
  {
    plugin->update(model_, data_);
    mj_step(model_, data_);
  }

  mjtNum vel6[6];
  mj_objectVelocity(model_, data_, mjOBJ_BODY, base_id, vel6, /*flg_local=*/1);
  EXPECT_NEAR(vel6[2], 1.0, 0.1);
}

TEST_F(BaseVelocityPluginTest, StaleCommandDecaysToZeroForce)
{
  plugin_node_->declare_parameter("cmd_timeout", 0.2);
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/1.0, /*vy=*/0.0, /*wz=*/0.0);

  plugin->update(model_, data_);
  const int base_id = mj_name2id(model_, mjOBJ_BODY, "base_link");
  ASSERT_GE(base_id, 0);
  ASSERT_GT(data_->xfrc_applied[base_id * 6 + 0], 0.0) << "sanity: force applied while command is fresh";

  // Wait past the timeout without publishing again.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  plugin->update(model_, data_);

  EXPECT_NEAR(data_->xfrc_applied[base_id * 6 + 0], 0.0, 1e-9)
      << "stale command should be treated as zero, producing no force";

  plugin->cleanup();
}
