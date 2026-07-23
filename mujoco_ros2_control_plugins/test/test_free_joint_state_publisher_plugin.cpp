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
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_msgs/msg/free_joint_state_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "free_joint_state_publisher_plugin.hpp"

namespace
{
// Two free bodies (free_a, free_b) plus a fixed reference body (ref_frame) offset and rotated
// from the world origin, used to exercise non-world reference frame transforms.
constexpr const char* kMjcf = R"(
<mujoco model="free_joint_state_publisher_test">
  <worldbody>
    <body name="ref_frame" pos="1 0 0" quat="0.70710678 0 0 0.70710678">
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
    <body name="free_a" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
    <body name="free_b" pos="2 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";
}  // namespace

class FreeJointStatePublisherPluginTest : public ::testing::Test
{
protected:
  using FreeJointStateArray = mujoco_ros2_control_msgs::msg::FreeJointStateArray;

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
    node_ = std::make_shared<rclcpp::Node>("free_joint_state_publisher_test_node");
    plugin_node_ = node_->create_sub_node("free_joint_state_publisher_plugin");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
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

  /// Sets the qpos/qvel of the free joint driving `body_name`. Fails the test if the body is
  /// not driven by a free joint.
  void setFreeJointState(const std::string& body_name, const mjtNum pos[3], const mjtNum quat[4],
                        const mjtNum linvel[3], const mjtNum angvel[3])
  {
    const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
    ASSERT_GE(body_id, 0) << body_name;
    for (int i = 0; i < model_->njnt; ++i)
    {
      if (model_->jnt_bodyid[i] == body_id && model_->jnt_type[i] == mjJNT_FREE)
      {
        const int qadr = model_->jnt_qposadr[i];
        const int vadr = model_->jnt_dofadr[i];
        data_->qpos[qadr + 0] = pos[0];
        data_->qpos[qadr + 1] = pos[1];
        data_->qpos[qadr + 2] = pos[2];
        data_->qpos[qadr + 3] = quat[0];
        data_->qpos[qadr + 4] = quat[1];
        data_->qpos[qadr + 5] = quat[2];
        data_->qpos[qadr + 6] = quat[3];
        data_->qvel[vadr + 0] = linvel[0];
        data_->qvel[vadr + 1] = linvel[1];
        data_->qvel[vadr + 2] = linvel[2];
        data_->qvel[vadr + 3] = angvel[0];
        data_->qvel[vadr + 4] = angvel[1];
        data_->qvel[vadr + 5] = angvel[2];
        return;
      }
    }
    FAIL() << body_name << " is not driven by a free joint";
  }

  /// Pre-declares and sets a parameter on plugin_node_ before init(), matching how launch
  /// parameter files set plugin parameters ahead of load_mujoco_plugins().
  template <typename T>
  void setParam(const std::string& name, const T& value)
  {
    if (!plugin_node_->has_parameter(name))
    {
      plugin_node_->declare_parameter(name, rclcpp::ParameterValue(value));
    }
    plugin_node_->set_parameter(rclcpp::Parameter(name, value));
  }

  /// Subscribes to `topic` on plugin_node_ (matching the publisher's node) and spins until a
  /// message arrives or the timeout elapses. Returns nullptr on timeout.
  FreeJointStateArray::SharedPtr waitForMessage(const std::string& topic,
                                                mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin& plugin)
  {
    std::atomic<bool> received{ false };
    FreeJointStateArray::SharedPtr last_msg;
    auto sub = plugin_node_->create_subscription<FreeJointStateArray>(
        topic, 10, [&](FreeJointStateArray::SharedPtr msg) {
          last_msg = msg;
          received = true;
        });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline)
    {
      plugin.update(model_, data_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return last_msg;
  }

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

private:
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(FreeJointStatePublisherPluginTest, InitSucceeds)
{
  mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

TEST_F(FreeJointStatePublisherPluginTest, WorldFrameDefaultsPublishAllFreeBodies)
{
  const mjtNum pos_a[3] = { 1.0, 2.0, 3.0 };
  const mjtNum quat_a[4] = { std::cos(0.5236), 0.0, 0.0, std::sin(0.5236) };  // 60 deg about Z
  const mjtNum linvel_a[3] = { 0.1, 0.2, 0.3 };
  const mjtNum angvel_a[3] = { 0.0, 0.0, 0.5 };
  setFreeJointState("free_a", pos_a, quat_a, linvel_a, angvel_a);

  const mjtNum pos_b[3] = { -1.0, 0.0, 0.5 };
  const mjtNum quat_b[4] = { 1.0, 0.0, 0.0, 0.0 };
  const mjtNum zero3[3] = { 0.0, 0.0, 0.0 };
  setFreeJointState("free_b", pos_b, quat_b, zero3, zero3);
  mj_forward(model_, data_);

  mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("free_joint_states", plugin);
  ASSERT_NE(msg, nullptr) << "No FreeJointStateArray received";
  ASSERT_EQ(msg->free_joints.size(), 2u);

  std::map<std::string, mujoco_ros2_control_msgs::msg::FreeJointState> by_name;
  for (const auto& entry : msg->free_joints)
  {
    by_name[entry.name] = entry;
  }
  ASSERT_EQ(by_name.count("free_a"), 1u);
  ASSERT_EQ(by_name.count("free_b"), 1u);

  const auto& a = by_name["free_a"];
  EXPECT_EQ(a.pose.header.frame_id, "");
  EXPECT_NEAR(a.pose.pose.position.x, pos_a[0], 1e-9);
  EXPECT_NEAR(a.pose.pose.position.y, pos_a[1], 1e-9);
  EXPECT_NEAR(a.pose.pose.position.z, pos_a[2], 1e-9);
  EXPECT_NEAR(a.pose.pose.orientation.w, quat_a[0], 1e-9);
  EXPECT_NEAR(a.pose.pose.orientation.x, quat_a[1], 1e-9);
  EXPECT_NEAR(a.pose.pose.orientation.y, quat_a[2], 1e-9);
  EXPECT_NEAR(a.pose.pose.orientation.z, quat_a[3], 1e-9);
  EXPECT_NEAR(a.twist.twist.linear.x, linvel_a[0], 1e-9);
  EXPECT_NEAR(a.twist.twist.linear.y, linvel_a[1], 1e-9);
  EXPECT_NEAR(a.twist.twist.linear.z, linvel_a[2], 1e-9);
  EXPECT_NEAR(a.twist.twist.angular.z, angvel_a[2], 1e-9);

  const auto& b = by_name["free_b"];
  EXPECT_NEAR(b.pose.pose.position.x, pos_b[0], 1e-9);
  EXPECT_NEAR(b.pose.pose.position.z, pos_b[2], 1e-9);

  plugin.cleanup();
}

TEST_F(FreeJointStatePublisherPluginTest, BodyNamesFilterRestrictsPublishedBodies)
{
  setParam("body_names", std::vector<std::string>{ "free_b" });

  mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("free_joint_states", plugin);
  ASSERT_NE(msg, nullptr);
  ASSERT_EQ(msg->free_joints.size(), 1u);
  EXPECT_EQ(msg->free_joints[0].name, "free_b");

  plugin.cleanup();
}

TEST_F(FreeJointStatePublisherPluginTest, NonWorldFrameTransformsPoseAndTwistRoundTrip)
{
  const mjtNum pos_a[3] = { 0.0, 0.0, 1.0 };
  const mjtNum quat_a[4] = { 1.0, 0.0, 0.0, 0.0 };
  const mjtNum linvel_a[3] = { 0.5, 0.0, 0.0 };
  const mjtNum angvel_a[3] = { 0.0, 0.0, 1.0 };
  setFreeJointState("free_a", pos_a, quat_a, linvel_a, angvel_a);
  const mjtNum zero3[3] = { 0.0, 0.0, 0.0 };
  const mjtNum ident_quat[4] = { 1.0, 0.0, 0.0, 0.0 };
  setFreeJointState("free_b", zero3, ident_quat, zero3, zero3);
  mj_forward(model_, data_);

  setParam("frame_id", std::string("ref_frame"));
  setParam("body_names", std::vector<std::string>{ "free_a" });

  mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto msg = waitForMessage("free_joint_states", plugin);
  ASSERT_NE(msg, nullptr);
  ASSERT_EQ(msg->free_joints.size(), 1u);
  const auto& entry = msg->free_joints[0];
  EXPECT_EQ(entry.pose.header.frame_id, "ref_frame");
  EXPECT_EQ(entry.twist.header.frame_id, "ref_frame");

  // Round trip test
  const int ref_id = mj_name2id(model_, mjOBJ_BODY, "ref_frame");
  ASSERT_GE(ref_id, 0);
  const mjtNum rel_pos[3] = { entry.pose.pose.position.x, entry.pose.pose.position.y,
                             entry.pose.pose.position.z };
  const mjtNum rel_quat[4] = { entry.pose.pose.orientation.w, entry.pose.pose.orientation.x,
                              entry.pose.pose.orientation.y, entry.pose.pose.orientation.z };
  mjtNum recomposed_pos[3];
  mjtNum recomposed_quat[4];
  mju_mulPose(recomposed_pos, recomposed_quat, data_->xpos + 3 * ref_id, data_->xquat + 4 * ref_id, rel_pos, rel_quat);
  EXPECT_NEAR(recomposed_pos[0], pos_a[0], 1e-6);
  EXPECT_NEAR(recomposed_pos[1], pos_a[1], 1e-6);
  EXPECT_NEAR(recomposed_pos[2], pos_a[2], 1e-6);
  EXPECT_NEAR(recomposed_quat[0], quat_a[0], 1e-6);
  EXPECT_NEAR(recomposed_quat[1], quat_a[1], 1e-6);
  EXPECT_NEAR(recomposed_quat[2], quat_a[2], 1e-6);
  EXPECT_NEAR(recomposed_quat[3], quat_a[3], 1e-6);

  // Same round trip for twist
  const mjtNum rel_linvel[3] = { entry.twist.twist.linear.x, entry.twist.twist.linear.y, entry.twist.twist.linear.z };
  const mjtNum rel_angvel[3] = { entry.twist.twist.angular.x, entry.twist.twist.angular.y,
                                entry.twist.twist.angular.z };
  mjtNum recomposed_linvel[3];
  mjtNum recomposed_angvel[3];
  mju_rotVecQuat(recomposed_linvel, rel_linvel, data_->xquat + 4 * ref_id);
  mju_rotVecQuat(recomposed_angvel, rel_angvel, data_->xquat + 4 * ref_id);
  EXPECT_NEAR(recomposed_linvel[0], linvel_a[0], 1e-6);
  EXPECT_NEAR(recomposed_linvel[1], linvel_a[1], 1e-6);
  EXPECT_NEAR(recomposed_linvel[2], linvel_a[2], 1e-6);
  EXPECT_NEAR(recomposed_angvel[2], angvel_a[2], 1e-6);

  plugin.cleanup();
}

TEST_F(FreeJointStatePublisherPluginTest, UnknownFrameIdFallsBackToWorld)
{
  const mjtNum pos_a[3] = { 3.0, 4.0, 5.0 };
  const mjtNum quat_a[4] = { 1.0, 0.0, 0.0, 0.0 };
  const mjtNum zero3[3] = { 0.0, 0.0, 0.0 };
  setFreeJointState("free_a", pos_a, quat_a, zero3, zero3);
  setFreeJointState("free_b", zero3, quat_a, zero3, zero3);
  mj_forward(model_, data_);

  setParam("frame_id", std::string("nonexistent_body"));
  setParam("body_names", std::vector<std::string>{ "free_a" });

  mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_)) << "init() must not fail on an unknown frame_id";

  auto msg = waitForMessage("free_joint_states", plugin);
  ASSERT_NE(msg, nullptr);
  ASSERT_EQ(msg->free_joints.size(), 1u);
  const auto& entry = msg->free_joints[0];
  EXPECT_EQ(entry.pose.header.frame_id, "") << "Should fall back to the world frame";
  EXPECT_NEAR(entry.pose.pose.position.x, pos_a[0], 1e-9);
  EXPECT_NEAR(entry.pose.pose.position.y, pos_a[1], 1e-9);
  EXPECT_NEAR(entry.pose.pose.position.z, pos_a[2], 1e-9);

  plugin.cleanup();
}

TEST_F(FreeJointStatePublisherPluginTest, InitRejectsUnknownBodyNameInFilter)
{
  setParam("body_names", std::vector<std::string>{ "nonexistent_body" });

  mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
}
