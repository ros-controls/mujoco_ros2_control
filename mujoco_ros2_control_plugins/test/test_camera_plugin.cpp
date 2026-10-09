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
#include "camera_plugin_test_helper.hpp"

using namespace std::chrono_literals;

class CameraPluginTest : public ::testing::Test
{
protected:
  static constexpr auto WAIT_TIMEOUT = 2s;
  static constexpr auto POLL_INTERVAL = 20ms;

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

  // Blocks until the rendering thread has finished initializing its GL context.
  void wait_for_rendering(mujoco_ros2_control_plugins::CameraPlugin& plugin)
  {
    const auto deadline = std::chrono::steady_clock::now() + WAIT_TIMEOUT;
    while (!plugin.is_rendering_available() && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
    // TODO: We could optionally skip the test in CI if this happens consistently. Though
    // for now we hope that failures due to uninitialized contexts will be extremely rare.
    // https://github.com/ros-controls/mujoco_ros2_control/issues/216
    ASSERT_TRUE(plugin.is_rendering_available()) << "OpenGL rendering unavailable in this environment!!! Failing...";
  }

  // Block until every subscription in the list has matched at least one publisher.
  template <typename... Subs>
  void wait_for_subscriber_match(const Subs&... subs)
  {
    auto all_matched = [&]() { return ((subs->get_publisher_count() > 0) && ...); };
    const auto deadline = std::chrono::steady_clock::now() + WAIT_TIMEOUT;
    while (!all_matched() && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
    ASSERT_TRUE(all_matched()) << "Subscribers never matched publishers within timeout";
  }

  // Calls a Trigger service and returns whether the call succeeded.
  bool call_trigger(const std::string& service_name)
  {
    auto client = node_->create_client<std_srvs::srv::Trigger>(service_name);
    if (!client->wait_for_service(WAIT_TIMEOUT))
    {
      return false;
    }
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(WAIT_TIMEOUT) != std::future_status::ready)
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

  // Wait for the rendering thread to come up
  wait_for_rendering(plugin);

  // Verify publishers were created
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/color"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/depth"), 1u);
  EXPECT_EQ(plugin_node_->count_publishers("/camera_plugin/test_cam/camera_info"), 1u);

  // Create subscribers for the color images and info
  std::atomic<bool> received_image{ false };
  std::atomic<bool> received_depth{ false };
  std::atomic<bool> received_info{ false };
  std::atomic<bool> received_expected_frame_id{ false };
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/test_cam/color", 1, [&](sensor_msgs::msg::Image::SharedPtr) { received_image = true; });
  auto depth_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/test_cam/depth", 1, [&](sensor_msgs::msg::Image::SharedPtr) { received_depth = true; });
  auto info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_plugin/test_cam/camera_info", 1, [&](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        received_info = true;
        received_expected_frame_id = (msg->header.frame_id == "test_cam_frame");
      });

  // Ensure publishers are connected to subscribers
  wait_for_subscriber_match(image_sub, depth_sub, info_sub);

  // Force a publish and verify we get results
  const auto deadline = std::chrono::steady_clock::now() + WAIT_TIMEOUT;
  while (!(received_image && received_depth && received_info && received_expected_frame_id) &&
         std::chrono::steady_clock::now() < deadline)
  {
    plugin.trigger_update();
    std::this_thread::sleep_for(POLL_INTERVAL);
  }

  ASSERT_TRUE(received_image);
  ASSERT_TRUE(received_depth);
  ASSERT_TRUE(received_info);
  ASSERT_TRUE(received_expected_frame_id);

  plugin.cleanup();
}

// Only polled cameras should expose a trigger service; streaming cameras should not.
TEST_F(CameraPluginTest, OnlyPolledCamerasCreateTriggerService)
{
  const std::string param_prefix = "mujoco_plugins." + plugin_node_->get_sub_namespace() + ".";
  plugin_node_->declare_parameter(param_prefix + "stream_cam.policy", std::string("streaming"));
  plugin_node_->declare_parameter(param_prefix + "poll_cam.policy", std::string("polled"));
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
  EXPECT_TRUE(poll_client->wait_for_service(WAIT_TIMEOUT));

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

  const std::string param_prefix = "mujoco_plugins." + plugin_node_->get_sub_namespace() + ".";
  plugin_node_->declare_parameter(param_prefix + "poll_cam.policy", std::string("polled"));

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_, []() { return 0; }));

  std::atomic<int> image_count{ 0 };
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/poll_cam/color", 10, [&](sensor_msgs::msg::Image::SharedPtr) { ++image_count; });

  // Wait for the rendering thread to come up
  wait_for_rendering(plugin);

  // Ensure publishers are connected to subscribers
  wait_for_subscriber_match(image_sub);

  // Without a trigger, running render cycles must not publish anything for a polled camera.
  for (auto i = 0; i < 5; ++i)
  {
    plugin.trigger_update();
    std::this_thread::sleep_for(POLL_INTERVAL);
  }
  EXPECT_EQ(image_count.load(), 0);

  // Trigger once; the next render cycle should publish exactly one image.
  ASSERT_TRUE(call_trigger("/camera_plugin/poll_cam/trigger"));
  plugin.trigger_update();
  {
    const auto deadline = std::chrono::steady_clock::now() + WAIT_TIMEOUT;
    while (image_count.load() < 1 && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
  }
  EXPECT_EQ(image_count.load(), 1);

  // The trigger is one-shot: subsequent render cycles without a new trigger publish nothing more.
  for (auto i = 0; i < 5; ++i)
  {
    plugin.trigger_update();
    std::this_thread::sleep_for(POLL_INTERVAL);
  }
  EXPECT_EQ(image_count.load(), 1);

  plugin.cleanup();
}

// ---------------------------------------------------------------------------------------
// Dropped-frame accounting
//
// update() must not overwrite a snapshot the rendering thread is still working on; when it
// has to skip, it consumes the slot and counts one lost frame. These tests force that
// "renderer still busy" state directly through CameraPluginTestHelper rather than racing a
// real render pass against the clock (e.g. via an oversized camera), so they are
// deterministic and do not depend on the test machine being slow enough -- or fast enough --
// to reproduce the race.
// ---------------------------------------------------------------------------------------

namespace
{
constexpr const char* SINGLE_STREAMING_CAMERA_MODEL = R"(<?xml version="1.0"?>
<mujoco model="single_camera">
  <worldbody>
    <light pos="0 0 3"/>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
    <camera name="cam" pos="0 -1 0.2" mode="fixed" resolution="64 48"/>
  </worldbody>
</mujoco>
)";
}  // namespace

// Sanity check on the real path: a small camera rendering well within its publish interval
// must never trip the guard, across several real slots (not just the first one).
TEST_F(CameraPluginTest, NoFramesAreDroppedAcrossSeveralRealIntervals)
{
  load_model(SINGLE_STREAMING_CAMERA_MODEL);
  // 50 Hz: fast enough that the loop below spans several real slots, slow enough that a
  // 64x48 render comfortably finishes within one.
  plugin_node_->declare_parameter("mujoco_plugins.camera_plugin.camera_publish_rate", 50.0);

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  wait_for_rendering(plugin);

  // The very first render pass pays one-off setup cost (e.g. the offscreen buffer resize)
  // that has nothing to do with whether the renderer can sustain the publish rate. Let that
  // cost land here, via the real async rendering thread, instead of in the timed loop below.
  // (Rendering happens off-thread; calling GL synchronously from this thread, as
  // trigger_update() does, is only safe when the context was forced onto EGL, which this
  // test does not do.)
  plugin.update(model_, data_);
  std::this_thread::sleep_for(100ms);

  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  while (std::chrono::steady_clock::now() < deadline)
  {
    plugin.update(model_, data_);
    std::this_thread::sleep_for(1ms);
  }

  EXPECT_EQ(plugin.dropped_frames(), 0u);
  plugin.cleanup();
}

// The regression this guards: without the in-flight check, update() would copy into the
// snapshot the rendering thread is reading, producing a frame blended from two sim times --
// corruption that still looks like a valid image. The counter is the observable proof that
// the slot was skipped instead.
TEST_F(CameraPluginTest, UpdateSkipsAndCountsExactlyOneDropPerElapsedIntervalWhileRenderIsInFlight)
{
  load_model(SINGLE_STREAMING_CAMERA_MODEL);
  // 1 kHz, so several intervals elapse over a short, deterministic busy loop.
  plugin_node_->declare_parameter("mujoco_plugins.camera_plugin.camera_publish_rate", 1000.0);

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  wait_for_rendering(plugin);

  // Force the state update() would see mid-render. The real rendering thread is left idle --
  // it only wakes once update() notifies it, which never happens on the path under test --
  // so this exercises nothing but the guard itself.
  mujoco_ros2_control_plugins::CameraPluginTestHelper::set_render_in_flight(plugin, true);

  std::size_t calls = 0;
  const auto deadline = std::chrono::steady_clock::now() + 50ms;
  while (std::chrono::steady_clock::now() < deadline)
  {
    plugin.update(model_, data_);
    ++calls;
  }

  const uint64_t dropped = plugin.dropped_frames();
  ASSERT_GT(calls, 500u) << "the loop did not run often enough to be meaningful";
  EXPECT_GT(dropped, 0u);

  // The point of consuming the slot: each lost frame is counted once, so the number of
  // drops is bounded by the elapsed intervals (~50 at 1 kHz over 50 ms) and not by the
  // number of update() calls. Without that, every call while in flight would increment.
  EXPECT_LT(dropped, static_cast<uint64_t>(calls))
      << "counted " << dropped << " drops over " << calls << " update() calls: the slot is not being consumed";
  EXPECT_LT(dropped, 150u) << "far more drops than intervals elapsed in 50 ms";

  // Once the render is no longer in flight, the next elapsed interval must publish normally
  // instead of continuing to drop.
  mujoco_ros2_control_plugins::CameraPluginTestHelper::set_render_in_flight(plugin, false);
  std::this_thread::sleep_for(2ms);
  plugin.update(model_, data_);
  EXPECT_EQ(plugin.dropped_frames(), dropped) << "a skip was counted after the render was no longer in flight";

  plugin.cleanup();
}

// Polled triggers must never be dropped, only deferred: a skip while the render is in
// flight has to re-arm the request rather than consume it.
TEST_F(CameraPluginTest, PollTriggerSurvivesASkipInsteadOfBeingLost)
{
  load_model(SINGLE_STREAMING_CAMERA_MODEL);
  const std::string param_prefix = "mujoco_plugins." + plugin_node_->get_sub_namespace() + ".";
  plugin_node_->declare_parameter(param_prefix + "cam.policy", std::string("polled"));

  mujoco_ros2_control_plugins::CameraPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  std::atomic<int> image_count{ 0 };
  auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
      "/camera_plugin/cam/color", 10, [&](sensor_msgs::msg::Image::SharedPtr) { ++image_count; });

  wait_for_rendering(plugin);
  wait_for_subscriber_match(image_sub);

  // Force the render-in-flight state, then trigger: the resulting skip must defer the
  // request, not drop it, and must not be counted as a dropped (streaming) frame.
  mujoco_ros2_control_plugins::CameraPluginTestHelper::set_render_in_flight(plugin, true);
  ASSERT_TRUE(call_trigger("/camera_plugin/cam/trigger"));
  plugin.update(model_, data_);
  EXPECT_EQ(image_count.load(), 0) << "the trigger should not have rendered while the guard held it off";
  EXPECT_EQ(plugin.dropped_frames(), 0u) << "a deferred poll is not a dropped streaming frame";

  // Once the render is no longer in flight, the deferred trigger must still fire exactly once.
  mujoco_ros2_control_plugins::CameraPluginTestHelper::set_render_in_flight(plugin, false);
  plugin.update(model_, data_);
  {
    const auto deadline = std::chrono::steady_clock::now() + WAIT_TIMEOUT;
    while (image_count.load() < 1 && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(POLL_INTERVAL);
    }
  }
  EXPECT_EQ(image_count.load(), 1);

  plugin.cleanup();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
