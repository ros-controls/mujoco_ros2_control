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
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "base_velocity_plugin.hpp"

namespace
{
// A free-floating sphere (nv = 6: 3 translational + 3 rotational DOFs), no gravity so the
// body only moves if the plugin's velocity override writes into qvel. Identity orientation
// means body-frame and world-frame vectors coincide at t=0, which keeps the arithmetic in
// most tests simple to reason about.
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

// Same body, but welded to the world (no <freejoint/>) -- used to verify that init() now
// fails outright, since a velocity override has nowhere to write without a free joint.
constexpr const char* kMjcfNoFreeJoint = R"(
<mujoco model="base_velocity_test_no_free">
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="base_link" pos="0 0 1">
      <inertial mass="1.0" pos="0 0 0" diaginertia="0.1 0.1 0.1"/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";

mjModel* compileModel(const char* xml)
{
  char error[1024] = { 0 };
  mjSpec* spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
  if (spec == nullptr)
  {
    ADD_FAILURE() << error;
    return nullptr;
  }
  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr)
  {
    ADD_FAILURE() << (mjs_getError(spec) ? mjs_getError(spec) : "mj_compile failed");
  }
  mj_deleteSpec(spec);
  return model;
}
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

    model_ = compileModel(kMjcf);
    ASSERT_NE(model_, nullptr);

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

  std::string paramName(const std::string& name)
  {
    return "mujoco_plugins." + plugin_node_->get_sub_namespace() + "." + name;
  }

  /// Index of the (only) joint's first qvel/qpos entry in this single-free-joint test model.
  int dofAdr() const
  {
    return model_->jnt_dofadr[0];
  }
  int qposAdr() const
  {
    return model_->jnt_qposadr[0];
  }

  /// NaN-fills this body's 6 qvel entries, mirroring the real system's per-cycle pre-fill
  /// (MujocoSystemInterface::write()) so tests can verify which entries the plugin leaves
  /// untouched versus which it overrides.
  void fillQvelNaN()
  {
    const int adr = dofAdr();
    for (int i = 0; i < 6; ++i)
    {
      data_->qvel[adr + i] = std::numeric_limits<double>::quiet_NaN();
    }
  }

  /// Declares "body" (defaulting to "base_link") and initializes a plugin instance with it.
  std::unique_ptr<mujoco_ros2_control_plugins::BaseVelocityPlugin>
  makeInitializedPlugin(const std::string& body_name = "base_link")
  {
    if (!plugin_node_->has_parameter(paramName("body")))
    {
      plugin_node_->declare_parameter(paramName("body"), body_name);
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
  plugin_node_->declare_parameter(paramName("body"), std::string("does_not_exist"));

  mujoco_ros2_control_plugins::BaseVelocityPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

TEST_F(BaseVelocityPluginTest, InitSucceedsForFreeJointBody)
{
  auto plugin = makeInitializedPlugin();
  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, InitFailsForBodyWithoutFreeJoint)
{
  mjModel* welded_model = compileModel(kMjcfNoFreeJoint);
  ASSERT_NE(welded_model, nullptr);
  mjData* welded_data = mj_makeData(welded_model);
  ASSERT_NE(welded_data, nullptr);
  mj_forward(welded_model, welded_data);

  plugin_node_->declare_parameter(paramName("body"), std::string("base_link"));
  mujoco_ros2_control_plugins::BaseVelocityPlugin plugin;
  // A velocity override has nowhere to write without a free joint, so init() must fail
  // outright rather than merely warn as the old force-servo design did.
  EXPECT_FALSE(plugin.init(plugin_node_, welded_model, welded_data));
  plugin.cleanup();

  mj_deleteData(welded_data);
  mj_deleteModel(welded_model);
}

TEST_F(BaseVelocityPluginTest, NoCommandRequestsZeroVelocity)
{
  auto plugin = makeInitializedPlugin();
  fillQvelNaN();

  plugin->pre_step(data_);

  const mjtNum* qvel = data_->qvel + dofAdr();
  EXPECT_DOUBLE_EQ(qvel[0], 0.0);
  EXPECT_DOUBLE_EQ(qvel[1], 0.0);
  EXPECT_DOUBLE_EQ(qvel[5], 0.0);

  // Vertical linear velocity and roll/pitch are left to gravity/contacts, never driven --
  // the real system's NaN pre-fill (mirrored here) must survive untouched.
  EXPECT_TRUE(std::isnan(qvel[2]));
  EXPECT_TRUE(std::isnan(qvel[3]));
  EXPECT_TRUE(std::isnan(qvel[4]));

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, CommandIsReportedAsOverride)
{
  auto plugin = makeInitializedPlugin();
  fillQvelNaN();

  publishTwist("cmd_vel", /*vx=*/1.0, /*vy=*/0.0, /*wz=*/0.3);
  plugin->pre_step(data_);

  const mjtNum* qvel = data_->qvel + dofAdr();
  // Identity orientation => body-x == world-x. This is a direct assignment, not a servo,
  // so the commanded value is reached immediately -- no convergence tolerance needed.
  EXPECT_NEAR(qvel[0], 1.0, 1e-9);
  EXPECT_NEAR(qvel[1], 0.0, 1e-9);
  EXPECT_NEAR(qvel[5], 0.3, 1e-9);

  EXPECT_TRUE(std::isnan(qvel[2]));
  EXPECT_TRUE(std::isnan(qvel[3]));
  EXPECT_TRUE(std::isnan(qvel[4]));

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, StaleCommandRequestsZeroVelocity)
{
  plugin_node_->declare_parameter(paramName("cmd_timeout"), 0.2);
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/1.0, /*vy=*/0.0, /*wz=*/0.0);

  plugin->pre_step(data_);
  const mjtNum* qvel = data_->qvel + dofAdr();
  ASSERT_GT(qvel[0], 0.0) << "sanity: velocity requested while command is fresh";

  // Wait past the timeout without publishing again.
  std::this_thread::sleep_for(std::chrono::milliseconds(400));
  plugin->pre_step(data_);

  EXPECT_NEAR(qvel[0], 0.0, 1e-9) << "stale command should be treated as zero";

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, LinearCommandIsClampedToMaxLinearVelocity)
{
  plugin_node_->declare_parameter(paramName("max_linear_velocity"), 0.2);
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/5.0, /*vy=*/0.0, /*wz=*/0.0);
  plugin->pre_step(data_);

  const mjtNum* qvel = data_->qvel + dofAdr();
  EXPECT_NEAR(qvel[0], 0.2, 1e-9);

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, YawCommandIsClampedToMaxYawRate)
{
  plugin_node_->declare_parameter(paramName("max_yaw_rate"), 0.3);
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/0.0, /*vy=*/0.0, /*wz=*/5.0);
  plugin->pre_step(data_);

  const mjtNum* qvel = data_->qvel + dofAdr();
  EXPECT_NEAR(qvel[5], 0.3, 1e-9);

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, UnsetLimitsDoNotClamp)
{
  // Neither max_linear_velocity nor max_yaw_rate is declared -- both default to +inf, so
  // even a very large command must pass through unclamped.
  auto plugin = makeInitializedPlugin();

  publishTwist("cmd_vel", /*vx=*/1000.0, /*vy=*/0.0, /*wz=*/1000.0);
  plugin->pre_step(data_);

  const mjtNum* qvel = data_->qvel + dofAdr();
  EXPECT_NEAR(qvel[0], 1000.0, 1e-6);
  EXPECT_NEAR(qvel[5], 1000.0, 1e-6);

  plugin->cleanup();
}

TEST_F(BaseVelocityPluginTest, BodyFrameCommandIsRotatedIntoFreeJointFrame)
{
  auto plugin = makeInitializedPlugin();

  // Yaw the body 90 degrees about world z before commanding, so body-frame +x now points
  // along world +y. A free joint's linear qvel is world-frame; if the plugin forgot to
  // rotate the commanded body-frame velocity by the body's orientation, this test would
  // see the command land on the wrong world axis.
  const int qpos_adr = qposAdr();
  data_->qpos[qpos_adr + 3] = std::cos(M_PI / 4.0);  // qw
  data_->qpos[qpos_adr + 4] = 0.0;                   // qx
  data_->qpos[qpos_adr + 5] = 0.0;                   // qy
  data_->qpos[qpos_adr + 6] = std::sin(M_PI / 4.0);  // qz
  mj_forward(model_, data_);

  publishTwist("cmd_vel", /*vx=*/1.0, /*vy=*/0.0, /*wz=*/0.0);
  plugin->pre_step(data_);

  const mjtNum* qvel = data_->qvel + dofAdr();
  EXPECT_NEAR(qvel[0], 0.0, 1e-6) << "world-x should be ~0 after a 90 deg yaw";
  EXPECT_NEAR(qvel[1], 1.0, 1e-6) << "commanded body-x should land on world-y";

  plugin->cleanup();
}
