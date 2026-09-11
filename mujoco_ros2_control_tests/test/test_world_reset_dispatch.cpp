// Copyright 2026 Bilal Gill
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

// End-to-end check of the on_reset plugin hook: the out-of-package TestPlugin is loaded
// through the `mujoco_plugins` parameter namespace and must have its on_reset() called
// when the reset_world service resets the simulation.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <thread>

#include <hardware_interface/version.h>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32.hpp>

#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <mujoco_ros2_control_msgs/srv/reset_world.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace
{

constexpr const char* kTestModel = R"(<?xml version="1.0"?>
<mujoco model="test_world_reset_dispatch">
  <option timestep="0.002"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)";

// Loads TestPlugin into the hardware node via the wildcard parameter file passed through the
// `pids_config_file` hardware parameter (the node auto-declares parameters from overrides).
constexpr const char* kPluginParams = R"(/**:
  ros__parameters:
    mujoco_plugins:
      test_plugin:
        type: "external_plugin_test/TestPlugin"
)";

const std::string kTestModelPath = "/tmp/test_world_reset_dispatch_model.xml";
const std::string kPluginParamsPath = "/tmp/test_world_reset_dispatch_params.yaml";

void write_file(const std::string& path, const char* content)
{
  std::ofstream file(path);
  file << content;
}

}  // namespace

class WorldResetDispatchTest : public ::testing::Test
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
    write_file(kTestModelPath, kTestModel);
    write_file(kPluginParamsPath, kPluginParams);
    interface_ = std::make_shared<mujoco_ros2_control::MujocoSystemInterface>();

    node_ = std::make_shared<rclcpp::Node>("world_reset_dispatch_test_node");
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
    if (interface_)
    {
      rclcpp_lifecycle::State inactive_state(0, "inactive");
      interface_->on_deactivate(inactive_state);
      interface_.reset();
    }
    node_.reset();
    executor_.reset();

    std::filesystem::remove(kTestModelPath);
    std::filesystem::remove(kPluginParamsPath);
  }

  hardware_interface::CallbackReturn initialize_interface()
  {
    hardware_interface::HardwareInfo info;
    info.name = "test_world_reset_dispatch";
    info.type = "system";
    info.hardware_parameters["mujoco_model"] = kTestModelPath;
    info.hardware_parameters["meshdir"] = "";
    info.hardware_parameters["headless"] = "true";
    info.hardware_parameters["disable_rendering"] = "true";
    info.hardware_parameters["pids_config_file"] = kPluginParamsPath;
#if ROS_DISTRO_HUMBLE
    return interface_->on_init(info);
#else
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = info;
    return interface_->on_init(params);
#endif
  }

  bool wait_until(std::function<bool()> condition, std::chrono::milliseconds timeout = std::chrono::seconds(5),
                  std::chrono::milliseconds poll_interval = std::chrono::milliseconds(10))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (condition())
        return true;
      std::this_thread::sleep_for(poll_interval);
    }
    return condition();
  }

  std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> interface_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(WorldResetDispatchTest, ResetWorldServiceDispatchesWorldResetToPlugins)
{
  ASSERT_EQ(initialize_interface(), hardware_interface::CallbackReturn::SUCCESS);

  // TestPlugin publishes its on_reset call count (latched) from its plugin sub-namespace.
  std::atomic<uint32_t> world_reset_count{ 0 };
  auto count_sub = node_->create_subscription<std_msgs::msg::UInt32>(
      "/test_plugin/world_reset_count", rclcpp::QoS(1).transient_local(),
      [&world_reset_count](const std_msgs::msg::UInt32& msg) { world_reset_count = msg.data; });

  auto reset_client =
      node_->create_client<mujoco_ros2_control_msgs::srv::ResetWorld>("/mujoco_ros2_control_node/reset_world");
  ASSERT_TRUE(reset_client->wait_for_service(std::chrono::seconds(5))) << "reset_world service not found";

  // No reset yet: the plugin must not have been notified.
  EXPECT_EQ(world_reset_count.load(), 0u);

  auto reset_future =
      reset_client->async_send_request(std::make_shared<mujoco_ros2_control_msgs::srv::ResetWorld::Request>());
  ASSERT_EQ(reset_future.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(reset_future.get()->success);

  EXPECT_TRUE(wait_until([&]() { return world_reset_count.load() == 1u; }))
      << "on_reset() was not dispatched to the loaded plugin on reset_world";
}
