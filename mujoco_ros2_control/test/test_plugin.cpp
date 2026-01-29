/**
 * Copyright (c) 2025, United States Government, as represented by the
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

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

class MujocoSystemInterfaceLoadingTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    plugin_loader_ = std::make_unique<pluginlib::ClassLoader<hardware_interface::SystemInterface>>(
        "hardware_interface", "hardware_interface::SystemInterface");
  }

  void TearDown() override
  {
    plugin_loader_.reset();
    rclcpp::shutdown();
  }

  std::unique_ptr<pluginlib::ClassLoader<hardware_interface::SystemInterface>> plugin_loader_;
};

TEST_F(MujocoSystemInterfaceLoadingTest, test_load_plugin)
{
  const std::string class_name = "mujoco_ros2_control/MujocoSystemInterface";

  // Pluginlib should find the class
  ASSERT_TRUE(plugin_loader_->isClassAvailable(class_name));

  // Construct a sim interface and verify it is not null
  std::shared_ptr<hardware_interface::SystemInterface> mujoco_sim = plugin_loader_->createSharedInstance(class_name);
  EXPECT_NE(mujoco_sim, nullptr);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
