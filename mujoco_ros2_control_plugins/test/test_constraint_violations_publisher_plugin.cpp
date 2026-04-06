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
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_msgs/msg/constraint_violations.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/constraint_violations_publisher_plugin.hpp"

class ConstraintViolationsPublisherPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    std::cerr << "[ConstraintViolations] SetUpTestSuite\n" << std::flush;
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    std::cerr << "[ConstraintViolations] rclcpp initialized\n" << std::flush;
  }

  static void TearDownTestSuite()
  {
    std::cerr << "[ConstraintViolations] TearDownTestSuite\n" << std::flush;
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ConstraintViolationsPublisherPluginTest, PublishesConstraintViolationForEqualityConstraint)
{
  std::cerr << "[ConstraintViolations] test body entered\n" << std::flush;
  constexpr auto kMjcf = R"(
<mujoco model="constraint_violation_plugin_test">
  <worldbody>
    <body name="body_a" pos="0 0 1">
      <joint name="joint_a" type="hinge" axis="0 0 1"/>
      <geom type="capsule" size="0.02 0.1"/>
    </body>
    <body name="body_b" pos="0 0 2">
      <joint name="joint_b" type="hinge" axis="0 0 1"/>
      <geom type="capsule" size="0.02 0.1"/>
    </body>
  </worldbody>
  <equality>
    <joint name="joint_lock" joint1="joint_a" joint2="joint_b" polycoef="0 1 0 0 0"/>
  </equality>
</mujoco>
)";

  char error[1024] = { 0 };
  std::cerr << "[ConstraintViolations] calling mj_parseXMLString\n" << std::flush;
  mjSpec* spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
  ASSERT_NE(spec, nullptr) << error;

  std::cerr << "[ConstraintViolations] calling mj_compile\n" << std::flush;
  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr)
  {
    const char* compile_error = mjs_getError(spec);
    std::string compile_error_msg = compile_error ? compile_error : "Unknown mj_compile error";
    mj_deleteSpec(spec);
    FAIL() << compile_error_msg;
  }
  mj_deleteSpec(spec);

  std::cerr << "[ConstraintViolations] calling mj_makeData\n" << std::flush;
  mjData* data = mj_makeData(model);
  ASSERT_NE(data, nullptr);

  std::cerr << "[ConstraintViolations] calling mj_forward\n" << std::flush;
  mj_forward(model, data);

  std::cerr << "[ConstraintViolations] mj_forward done, nefc=" << data->nefc
            << " efc_type=" << static_cast<void*>(data->efc_type) << " efc_id=" << static_cast<void*>(data->efc_id)
            << " efc_pos=" << static_cast<void*>(data->efc_pos) << "\n"
            << std::flush;

  int first_equality_row = -1;
  for (int i = 0; i < data->nefc; ++i)
  {
    std::cerr << "[ConstraintViolations] efc row " << i << " type=" << data->efc_type[i] << "\n" << std::flush;
    if (data->efc_type[i] == mjCNSTR_EQUALITY)
    {
      first_equality_row = i;
      break;
    }
  }
  std::cerr << "[ConstraintViolations] first_equality_row=" << first_equality_row << "\n" << std::flush;
  ASSERT_GE(first_equality_row, 0);

  const int eq_id = data->efc_id[first_equality_row];
  std::cerr << "[ConstraintViolations] eq_id=" << eq_id << "\n" << std::flush;
  const double injected_equality_residual = 0.4321;
  data->efc_pos[first_equality_row] = injected_equality_residual;
  const double expected_violation = injected_equality_residual;

  std::cerr << "[ConstraintViolations] creating rclcpp node\n" << std::flush;
  auto node = std::make_shared<rclcpp::Node>("constraint_violations_plugin_test_node");
  std::cerr << "[ConstraintViolations] rclcpp node created, calling create_sub_node\n" << std::flush;
  auto plugin_node = node->create_sub_node("constraint_violations_plugin");
  std::cerr << "[ConstraintViolations] sub_node created, declaring parameter\n" << std::flush;
  plugin_node->declare_parameter("publish_rate", 1000);
  std::cerr << "[ConstraintViolations] parameter declared\n" << std::flush;

  std::cerr << "[ConstraintViolations] calling plugin.init()\n" << std::flush;
  mujoco_ros2_control_plugins::ConstraintViolationsPublisherPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node, model, data));
  std::cerr << "[ConstraintViolations] plugin.init() done, sleeping 100ms\n" << std::flush;
  plugin_node->get_clock()->sleep_for(
      std::chrono::milliseconds(100));  // give the plugin some time to initialize the publisher

  bool received = false;
  mujoco_ros2_control_msgs::msg::ConstraintViolations msg;
  auto subscription = plugin_node->create_subscription<mujoco_ros2_control_msgs::msg::ConstraintViolations>(
      "~/constraint_violations", 10, [&](const mujoco_ros2_control_msgs::msg::ConstraintViolations::SharedPtr incoming) {
        msg = *incoming;
        received = true;
      });

  std::cerr << "[ConstraintViolations] calling plugin.update()\n" << std::flush;
  plugin.update(model, data);
  std::cerr << "[ConstraintViolations] plugin.update() done, waiting for message\n" << std::flush;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (!received && std::chrono::steady_clock::now() < deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  EXPECT_TRUE(received);
  ASSERT_EQ(msg.names.size(), 1u);
  ASSERT_EQ(msg.types.size(), 1u);
  ASSERT_EQ(msg.violations.size(), 1u);

  EXPECT_EQ(msg.names[0], "joint_lock");
  EXPECT_EQ(msg.types[0], "joint");
  EXPECT_DOUBLE_EQ(msg.violations[0], expected_violation);

  EXPECT_EQ(eq_id, 0);

  plugin.cleanup();
  subscription.reset();

  mj_deleteData(data);
  mj_deleteModel(model);
}
