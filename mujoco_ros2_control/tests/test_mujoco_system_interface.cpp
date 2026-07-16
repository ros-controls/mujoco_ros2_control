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
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <thread>

#include <hardware_interface/version.h>
#include <mujoco/mujoco.h>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mujoco_ros2_control/mujoco_system_interface.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

namespace
{

// Pendulum model with framepos/framequat sensors on a site at the pendulum body origin.
constexpr const char* kTestModel = R"(<?xml version="1.0"?>
<mujoco model="test_system_interface">
  <option timestep="0.002"/>

  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="1"/>
      <site name="pendulum_site"/>
    </body>
  </worldbody>

  <sensor>
    <framepos name="pose_sensor_pos" objtype="site" objname="pendulum_site"/>
    <framequat name="pose_sensor_quat" objtype="site" objname="pendulum_site"/>
  </sensor>
</mujoco>
)";

// Write to disk for testing
const std::string kTestModelPath = "/tmp/test_mujoco_system_interface_model.xml";
void write_test_model()
{
  std::ofstream file(kTestModelPath);
  file << kTestModel;
  file.close();
}

}  // namespace

class MujocoSystemInterfaceTest : public ::testing::Test
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
    write_test_model();
    interface_ = std::make_shared<mujoco_ros2_control::MujocoSystemInterface>();
  }

  void TearDown() override
  {
    // Deactivate interface before destroying to stop threads cleanly
    if (interface_)
    {
      rclcpp_lifecycle::State inactive_state(0, "inactive");
      interface_->on_deactivate(inactive_state);
      interface_.reset();
    }

    // Clean up test files if present
    if (std::filesystem::exists(kTestModelPath))
    {
      std::filesystem::remove(kTestModelPath);
    }
  }

  // Create the hardware info to initialize the interface with, in headless mode.
  hardware_interface::HardwareInfo create_hardware_info()
  {
    hardware_interface::HardwareInfo info;
    info.name = "test_mujoco_system_interface";
    info.type = "system";
    info.hardware_parameters["mujoco_model"] = kTestModelPath;
    info.hardware_parameters["meshdir"] = "";
    info.hardware_parameters["headless"] = "true";
    info.hardware_parameters["disable_rendering"] = "true";
    return info;
  }

  // Initialize the interface with the given hardware info.
  hardware_interface::CallbackReturn initialize_interface(const hardware_interface::HardwareInfo& hardware_info)
  {
#if ROS_DISTRO_HUMBLE
    return interface_->on_init(hardware_info);
#else
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = hardware_info;
    return interface_->on_init(params);
#endif
  }

  // Helper function to poll a condition until it returns true or the timeout expires.
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
};

TEST_F(MujocoSystemInterfaceTest, PoseSensorStateInterfacesRead)
{
  // Register a pose sensor mapped to the framepos/framequat sensors in the MJCF model.
  auto hardware_info = create_hardware_info();
  hardware_interface::ComponentInfo sensor_info;
  sensor_info.name = "pose_sensor";
  sensor_info.parameters["mujoco_type"] = "pose";
  for (const auto* interface_name :
       { "position.x", "position.y", "position.z", "orientation.x", "orientation.y", "orientation.z", "orientation.w" })
  {
    hardware_interface::InterfaceInfo interface_info;
    interface_info.name = interface_name;
    sensor_info.state_interfaces.push_back(interface_info);
  }
  hardware_info.sensors.push_back(sensor_info);

  ASSERT_EQ(initialize_interface(hardware_info), hardware_interface::CallbackReturn::SUCCESS);

  // Wait for the model to load and for the physics loop to advance, so that the
  // simulation has computed and published sensor data at least once.
  mjModel* model = nullptr;
  mjData* data = nullptr;
  ASSERT_TRUE(wait_until([&]() {
    interface_->get_model(model);
    interface_->get_data(data);
    return model != nullptr && data != nullptr && data->time > 0.0;
  })) << "Simulation did not start stepping";

  // read() should pull the latest sensor values into the exported state interfaces.
  interface_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002));

  // The pose sensor is the only component registered, so its interfaces are exported
  // in registration order: position.x/y/z, orientation.x/y/z/w.
  const auto state_interfaces = interface_->export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 7u);
  ASSERT_EQ(state_interfaces[0].get_name(), "pose_sensor/position.x");
  ASSERT_EQ(state_interfaces[1].get_name(), "pose_sensor/position.y");
  ASSERT_EQ(state_interfaces[2].get_name(), "pose_sensor/position.z");
  ASSERT_EQ(state_interfaces[3].get_name(), "pose_sensor/orientation.x");
  ASSERT_EQ(state_interfaces[4].get_name(), "pose_sensor/orientation.y");
  ASSERT_EQ(state_interfaces[5].get_name(), "pose_sensor/orientation.z");
  ASSERT_EQ(state_interfaces[6].get_name(), "pose_sensor/orientation.w");

#if ROS_DISTRO_HUMBLE
  const double position_z = state_interfaces[2].get_value();
  const double qx = state_interfaces[3].get_value();
  const double qy = state_interfaces[4].get_value();
  const double qz = state_interfaces[5].get_value();
  const double qw = state_interfaces[6].get_value();
#else
  const double position_z = state_interfaces[2].get_optional().value();
  const double qx = state_interfaces[3].get_optional().value();
  const double qy = state_interfaces[4].get_optional().value();
  const double qz = state_interfaces[5].get_optional().value();
  const double qw = state_interfaces[6].get_optional().value();
#endif

  // The site sits at the pendulum body origin at height 1.0, so the pose should be
  // a valid unit quaternion and a position fixed at the hinge point.
  const double tol = 1e-3;
  const double quat_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  EXPECT_NEAR(quat_norm, 1.0, tol);
  EXPECT_NEAR(position_z, 1.0, tol);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
