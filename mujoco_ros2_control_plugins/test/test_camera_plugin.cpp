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
#include <std_srvs/srv/trigger.hpp>

#include "camera_plugin.hpp"

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

  // Blocks until the rendering thread has finished initializing its GL context, or skip
  // the test if the context never comes up because of a bad CI instance...
  void wait_for_rendering(mujoco_ros2_control_plugins::CameraPlugin& plugin)
  {
    for (int i = 0; i < 50 && !plugin.is_rendering_available(); ++i)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!plugin.is_rendering_available())
    {
      plugin.cleanup();
      GTEST_SKIP() << "OpenGL rendering unavailable in this environment!!! Skipping tests...";
    }
  }

  // Block until every subscription in the list has matched at least one publisher.
  template <typename... Subs>
  void wait_for_subscriber_match(const Subs&... subs)
  {
    auto all_matched = [&]() { return ((subs->get_publisher_count() > 0) && ...); };
    for (int i = 0; i < 50 && !all_matched(); ++i)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(all_matched()) << "Subscribers never matched publishers within 2 s";
  }

  // Calls a Trigger service and returns whether the call succeeded.
  bool call_trigger(const std::string& service_name)
  {
    auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
      return false;
    }
    return future.get()->success;
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

  // Wait for the rendering thread to come up... If it doesn't we're going to skip this
  wait_for_rendering(plugin);

  // Verify publishers were created
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/color"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/depth"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/camera_info"), 1u);

  // Create subscribers for the color images and info
  std::atomic<bool> received_image{ false };
  std::atomic<bool> received_depth{ false };
  std::atomic<bool> received_info{ false };
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/test_cam/color", 1, [&](sensor_msgs::msg::Image::SharedPtr) { received_image = true; });
  auto depth_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/test_cam/depth", 1, [&](sensor_msgs::msg::Image::SharedPtr) { received_depth = true; });
  auto info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_plugin/test_cam/camera_info", 1, [&](sensor_msgs::msg::CameraInfo::SharedPtr) { received_info = true; });

  // Ensure publishers are connected to subscribers
  wait_for_subscriber_match(image_sub, depth_sub, info_sub);

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

// Only polled cameras should expose a trigger service; streaming cameras should not.
TEST_F(CameraPluginTest, OnlyPolledCamerasCreateTriggerService)
{
  plugin_node_->declare_parameter("mujoco_plugins.mujoco_camera_plugin.stream_cam.policy", std::string("streaming"));
  plugin_node_->declare_parameter("mujoco_plugins.mujoco_camera_plugin.poll_cam.policy", std::string("polled"));
  load_model(R"(<?xml version="1.0"?>
<mujoco model="two_cameras">
  <worldbody>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
    <camera name="stream_cam" pos="0 -1 1" xyaxes="1 0 0 0 0.707 0.707"
            fovy="60" resolution="64 48"/>
    <camera name="poll_cam" pos="0 1 1" xyaxes="-1 0 0 0 0.707 -0.707"
            fovy="60" resolution="64 48"/>
  </worldbody>
</mujoco>
)");
  ASSERT_EQ(model_->ncam, 2);

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_, []() { return 0; }));

  // The polled camera's trigger service must come up.
  auto poll_client = node_->create_client<std_srvs::srv::Trigger>("/camera_plugin/poll_cam/trigger");
  EXPECT_TRUE(poll_client->wait_for_service(std::chrono::seconds(2)));

  // The streaming camera must not have a trigger service registered.
  const auto services = node_->get_service_names_and_types();
  EXPECT_EQ(services.count("/camera_plugin/stream_cam/trigger"), 0u);

  // Both cameras still publish their image/info topics.
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/stream_cam/color"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/poll_cam/color"), 1u);

  plugin.cleanup();
}

// A polled camera must publish only in response to a trigger, never on the streaming clock,
// and exactly once per trigger.
TEST_F(CameraPluginTest, PolledCameraPublishesOncePerTrigger)
{
  load_model(R"(<?xml version="1.0"?>
<mujoco model="polled_only">
  <worldbody>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
    <camera name="poll_cam" pos="0 -1 1" xyaxes="1 0 0 0 0.707 0.707"
            fovy="60" resolution="64 48"/>
  </worldbody>
</mujoco>
)");
  ASSERT_EQ(model_->ncam, 1);

  plugin_node_->declare_parameter("mujoco_plugins.mujoco_camera_plugin.poll_cam.policy", std::string("polled"));

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_, []() { return 0; }));

  std::atomic<int> image_count{ 0 };
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/poll_cam/color", 10, [&](sensor_msgs::msg::Image::SharedPtr) { ++image_count; });

  // Wait for the rendering thread to come up... If it doesn't we're going to skip this
  wait_for_rendering(plugin);

  // Ensure publishers are connected to subscribers
  wait_for_subscriber_match(image_sub);

  // Without a trigger, running render cycles must not publish anything for a polled camera.
  for (auto i = 0; i < 5; ++i)
  {
    plugin.trigger_update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(image_count.load(), 0);

  // Trigger once; the next render cycle should publish exactly one image.
  ASSERT_TRUE(call_trigger("/camera_plugin/poll_cam/trigger"));
  plugin.trigger_update();
  for (auto i = 0; i < 20 && image_count.load() < 1; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(image_count.load(), 1);

  // The trigger is one-shot: subsequent render cycles without a new trigger publish nothing more.
  for (auto i = 0; i < 5; ++i)
  {
    plugin.trigger_update();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(image_count.load(), 1);

  plugin.cleanup();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
