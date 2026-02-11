// Copyright (C) 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Julia Jia

#include <gtest/gtest.h>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <rclcpp/rclcpp.hpp>

TEST(MinimalTest, NodeCreation)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("test_node");
  ASSERT_NE(node, nullptr);
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
