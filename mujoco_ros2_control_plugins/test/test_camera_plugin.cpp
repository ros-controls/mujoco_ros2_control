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
#include <filesystem>
#include <fstream>
#include <thread>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include <mujoco_ros2_control_plugins/camera_plugin.hpp>

class CameraPluginTest : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("test_camera_plugin_node");
    plugin_node_ = node_->create_sub_node("camera_plugin");
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    if (executor_)
    {
      executor_->cancel();
    }
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }

    executor_.reset();
    plugin_node_.reset();
    node_.reset();

    if (data_)
    {
      mj_deleteData(data_);
      data_ = nullptr;
    }
    if (model_)
    {
      mj_deleteModel(model_);
      model_ = nullptr;
    }

    for (const auto& path : temp_files_)
    {
      if (std::filesystem::exists(path))
      {
        std::filesystem::remove(path);
      }
    }
  }

  void load_model(const std::string& xml)
  {
    auto path = "/tmp/test_camera_plugin_" + std::to_string(temp_files_.size()) + ".xml";
    {
      std::ofstream f(path);
      f << xml;
    }
    temp_files_.push_back(path);

    char error[1000] = "";
    model_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << "mj_loadXML failed: " << error;
    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
  }

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;
  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  std::vector<std::string> temp_files_;
};

// Verify init returns true and does not start a render thread when model has no cameras.
TEST_F(CameraPluginTest, InitSucceedsWithNoCameras)
{
  load_model(R"(<?xml version="1.0"?>
<mujoco model="no_cameras">
  <worldbody>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
)");

  ASSERT_EQ(model_->ncam, 0);

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.update(model_, data_);
  plugin.cleanup();
}

// Verify init finds cameras and returns true, ensure the egl context does the rendering.
TEST_F(CameraPluginTest, InitAndPublish)
{
  load_model(R"(<?xml version="1.0"?>
<mujoco model="with_camera">
  <worldbody>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
    <camera name="test_cam" pos="0 -1 1" xyaxes="1 0 0 0 0.707 0.707"
            fovy="60" resolution="320 240"/>
  </worldbody>
</mujoco>
)");

  ASSERT_EQ(model_->ncam, 1);
  mujoco_ros2_control_plugins::CameraPlugin plugin;
  // GLFW is not initialized, and No OpenGL framebuffer will be available so we make the init fall back on to EGL.
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_, []() { return 0; }));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Verify publishers were created
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/color"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/depth"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/camera_info"), 1u);

  // Create subscribers for the color images and info
  std::atomic<bool> received_image{ false };
  std::atomic<bool> received_depth{ false };
  std::atomic<bool> received_info{ false };
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/test_cam/color", 1, [&](sensor_msgs::msg::Image::SharedPtr msg) { received_image = true; });
  auto depth_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/test_cam/depth", 1, [&](sensor_msgs::msg::Image::SharedPtr msg) { received_depth = true; });
  auto info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_plugin/test_cam/camera_info", 1,
      [&](sensor_msgs::msg::CameraInfo::SharedPtr msg) { received_info = true; });

  // Force a publish and verify we get results
  plugin.trigger_update();
  for (auto i = 0; i < 20; ++i)
  {
    if (received_image && received_depth && received_info)
      break;
    plugin.trigger_update();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ASSERT_TRUE(received_image);
  ASSERT_TRUE(received_depth);
  ASSERT_TRUE(received_info);

  plugin.cleanup();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
