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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include <hardware_interface/version.h>
#include <mujoco/mujoco.h>
#include <hardware_interface/hardware_info.hpp>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <mujoco_ros2_control/sim_display_text.hpp>
#include <rclcpp/rclcpp.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

class HeadlessInitTest : public ::testing::Test
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
    // Create a simple MuJoCo model
    create_test_model();

    // Initialize hardware interface
    hardware_info_ = create_hardware_info();
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

    // Clean up test file
    if (std::filesystem::exists(test_model_path_))
    {
      std::filesystem::remove(test_model_path_);
    }
  }

  void create_test_model()
  {
    // Create a simple MuJoCo XML with two bodies and a floor
    test_model_path_ = "/tmp/test_headless_init_model.xml";
    std::ofstream file(test_model_path_);
    file << R"(<?xml version="1.0"?>
<mujoco model="test_headless_init">
  <option timestep="0.002"/>

  <size nconmax="100"/>

  <worldbody>
    <!-- Floor as infinite plane directly in worldbody (static by default) -->
    <geom name="floor_geom" type="plane" size="0 0 1"
          contype="1" conaffinity="1"/>

    <body name="box1" pos="0 0 0.1">
      <freejoint name="box1_joint"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box1_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>

    <body name="box2" pos="0.2 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box2_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
      <site name="site1"/>
    </body>
  </worldbody>

  <actuator>
    <velocity name="actuator1" site="site1"/>
  </actuator>

  <sensor>
    <framepos name="pose_sensor_pos" objtype="site" objname="site1"/>
    <framequat name="pose_sensor_quat" objtype="site" objname="site1"/>
  </sensor>
</mujoco>
)";
    file.close();
  }

  hardware_interface::HardwareInfo create_hardware_info()
  {
    hardware_interface::HardwareInfo info;
    info.name = "test_mujoco";
    info.type = "system";
    info.hardware_parameters["mujoco_model"] = test_model_path_;
    info.hardware_parameters["meshdir"] = "";
    info.hardware_parameters["headless"] = "true";           // Enable headless mode for CI compatibility
    info.hardware_parameters["disable_rendering"] = "true";  // Disable cameras/lidar to avoid OpenGL issues in tests

    return info;
  }

  std::string test_model_path_;
  hardware_interface::HardwareInfo hardware_info_;
  std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> interface_;
};

TEST_F(HeadlessInitTest, HeadlessInitialization)
{
  // Test that MujocoSystemInterface can be initialized in headless mode
#if ROS_DISTRO_HUMBLE
  auto result = interface_->on_init(hardware_info_);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  auto result = interface_->on_init(params);
#endif
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Check that the data and model are available, meaning initializing was successful.
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(1);
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Model failed to initialize within timeout";

  // Test that we can export state interfaces without crashing
  auto state_interfaces = interface_->export_state_interfaces();
  EXPECT_GE(state_interfaces.size(), 0);
}

// Verify that on_activate() returns SUCCESS and does not start an OpenGL rendering thread when
// the model has no <camera> elements. Without the fix, glfwInit()/gladLoadGL would be called
// unconditionally and could crash on headless hosts that have GLFW but no usable GL driver.
TEST_F(HeadlessInitTest, HeadlessActivateWithoutCameras)
{
  hardware_interface::HardwareInfo info;
  info.name = "test_mujoco_no_cameras";
  info.type = "system";
  info.hardware_parameters["mujoco_model"] = test_model_path_;
  info.hardware_parameters["meshdir"] = "";
  info.hardware_parameters["headless"] = "true";
  // Intentionally omit "disable_rendering" — the fix must not rely on that workaround.

#if ROS_DISTRO_HUMBLE
  auto init_result = interface_->on_init(info);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  auto init_result = interface_->on_init(params);
#endif
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  // Wait for model and data to be available.
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(1);
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Data failed to initialize within timeout";

  // ncam == 0 for the test model; on_activate must not launch a GLFW/OpenGL thread.
  rclcpp_lifecycle::State active_state(0, "active");
  auto activate_result = interface_->on_activate(active_state);
  EXPECT_EQ(activate_result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(HeadlessInitTest, SpeedFactorParamInitialization)
{
  hardware_info_.hardware_parameters["sim_speed_factor"] = "0.5";

#if ROS_DISTRO_HUMBLE
  auto result = interface_->on_init(hardware_info_);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  auto result = interface_->on_init(params);
#endif
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Wait for the model to load so the physics thread starts.
  auto start = std::chrono::steady_clock::now();
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Data failed to initialize within timeout";

  // Let the physics thread run a few steps to confirm sim_speed_factor does not
  // cause a crash and the sim advances time.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Re-snapshot data after physics has had time to run (get_data copies the current state).
  interface_->get_data(test_data);

  // sim_time should have advanced from zero
  EXPECT_GT(test_data->time, 0.0) << "Simulation time did not advance with sim_speed_factor=0.5";
}

TEST_F(HeadlessInitTest, PoseSensorStateInterfacesRead)
{
  // Register a pose sensor mapped to the framepos/framequat sensors in the MJCF model.
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
  hardware_info_.sensors.push_back(sensor_info);

#if ROS_DISTRO_HUMBLE
  auto result = interface_->on_init(hardware_info_);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  auto result = interface_->on_init(params);
#endif
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Wait for the model to load and for the physics loop to advance, so that the
  // simulation has computed and published sensor data at least once.
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(2);
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr && test_data->time > 0.0)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Data failed to initialize within timeout";
  ASSERT_GT(test_data->time, 0.0) << "Simulation time did not advance within timeout";

  // read() should pull the latest sensor values into the exported state interfaces.
  interface_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002));

  // The pose sensor is the only component registered, so its interfaces are exported
  // in registration order: position.x/y/z, orientation.x/y/z/w.
  const auto state_interfaces = interface_->export_state_interfaces();
  ASSERT_EQ(state_interfaces.size(), 7u);
  ASSERT_EQ(state_interfaces[0].get_name(), "pose_sensor/position.x");
  ASSERT_EQ(state_interfaces[6].get_name(), "pose_sensor/orientation.w");

#if ROS_DISTRO_HUMBLE
  const double position_x = state_interfaces[0].get_value();
  const double qx = state_interfaces[3].get_value();
  const double qy = state_interfaces[4].get_value();
  const double qz = state_interfaces[5].get_value();
  const double qw = state_interfaces[6].get_value();
#else
  const double position_x = state_interfaces[0].get_optional().value();
  const double qx = state_interfaces[3].get_optional().value();
  const double qy = state_interfaces[4].get_optional().value();
  const double qz = state_interfaces[5].get_optional().value();
  const double qw = state_interfaces[6].get_optional().value();
#endif

  // The site sits on a body that starts upright at x=0.2, so the pose should be a
  // unit quaternion and a position near the initial pose.
  const double tol = 1e-3;
  const double quat_norm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  EXPECT_NEAR(quat_norm, 1.0, tol);
  EXPECT_NEAR(position_x, 0.2, tol);
}

TEST(SimDisplayTextTest, ComposesAllRowsWhenRunning)
{
  const auto [title, content] = mujoco_ros2_control::compose_sim_display_text(
      /*running=*/true, /*step_count=*/1234, /*sim_time=*/2.5,
      /*desired_pct=*/50.0, /*actual_pct=*/48.3, /*ncon=*/7);

  EXPECT_EQ(title, "Status\nSteps\nSim Time\nDesired Speed\nActual Speed\nContacts");
  EXPECT_EQ(content, "Running\n1234\n2.500 s\n50.0%\n48.3%\n7");

  // Title and content must line up row-for-row, otherwise the overlay is misaligned.
  const auto count_rows = [](const std::string& s) { return std::count(s.begin(), s.end(), '\n'); };
  EXPECT_EQ(count_rows(title), count_rows(content));
}

TEST(SimDisplayTextTest, ReportsPausedStatus)
{
  const auto [title, content] = mujoco_ros2_control::compose_sim_display_text(
      /*running=*/false, /*step_count=*/0, /*sim_time=*/0.0,
      /*desired_pct=*/100.0, /*actual_pct=*/0.0, /*ncon=*/0);

  EXPECT_EQ(content, "Paused\n0\n0.000 s\n100.0%\n0.0%\n0");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
