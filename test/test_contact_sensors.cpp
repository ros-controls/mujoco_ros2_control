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
#include <mujoco/mujoco.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <hardware_interface/hardware_info.hpp>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class ContactSensorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }

    // Create a simple MuJoCo model with two bodies that can contact
    create_test_model();

    // Initialize hardware interface
    hardware_info_ = create_hardware_info();
    interface_ = std::make_shared<mujoco_ros2_control::MujocoSystemInterface>();

    // Initialize the interface
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = hardware_info_;
    auto result = interface_->on_init(params);
    ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);
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

    // Clean up ROS
    rclcpp::shutdown();

    // Clean up test file
    if (std::filesystem::exists(test_model_path_))
    {
      std::filesystem::remove(test_model_path_);
    }
  }

  void create_test_model()
  {
    // Create a simple MuJoCo XML with two bodies and a floor
    test_model_path_ = "/tmp/test_contact_model.xml";
    std::ofstream file(test_model_path_);
    file << R"(<?xml version="1.0"?>
<mujoco model="test_contact">
  <option timestep="0.002"/>

  <size nconmax="100"/>

  <worldbody>
    <!-- Floor as infinite plane directly in worldbody (static by default) -->
    <!-- Plane size: [half_x half_y spacing] where spacing is for rendering grid -->
    <geom name="floor_geom" type="plane" size="0 0 1"
          contype="1" conaffinity="1"/>

    <body name="box1" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box1_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>

    <body name="box2" pos="0.2 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box2_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>
  </worldbody>
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

    // Add contact sensors
    // Note: floor_geom is in worldbody, so it belongs to the "world" body (body ID 0)
    hardware_interface::ComponentInfo contact1;
    contact1.name = "box1_contact";
    contact1.type = "sensor";
    contact1.parameters["mujoco_type"] = "contact";
    contact1.parameters["body1_name"] = "box1";
    contact1.parameters["body2_name"] = "world";  // worldbody geoms belong to "world"
    hardware_interface::InterfaceInfo contact1_if;
    contact1_if.name = "contact";
    contact1.state_interfaces.push_back(contact1_if);
    hardware_interface::InterfaceInfo contact1_raw_if;
    contact1_raw_if.name = "contact_raw";
    contact1.state_interfaces.push_back(contact1_raw_if);
    info.sensors.push_back(contact1);

    hardware_interface::ComponentInfo contact2;
    contact2.name = "box2_contact";
    contact2.type = "sensor";
    contact2.parameters["mujoco_type"] = "contact";
    contact2.parameters["body1_name"] = "box2";
    contact2.parameters["body2_name"] = "world";  // worldbody geoms belong to "world"
    hardware_interface::InterfaceInfo contact2_if;
    contact2_if.name = "contact";
    contact2.state_interfaces.push_back(contact2_if);
    hardware_interface::InterfaceInfo contact2_raw_if;
    contact2_raw_if.name = "contact_raw";
    contact2.state_interfaces.push_back(contact2_raw_if);
    info.sensors.push_back(contact2);

    // Gait consumer (debounced) for box1 contact
    hardware_interface::ComponentInfo contact1_gait;
    contact1_gait.name = "box1_contact_gait";
    contact1_gait.type = "sensor";
    contact1_gait.parameters["mujoco_type"] = "contact";
    contact1_gait.parameters["body1_name"] = "box1";
    contact1_gait.parameters["body2_name"] = "world";
    contact1_gait.parameters["contact_consumer"] = "gait";
    contact1_gait.parameters["debounce_on_steps"] = "3";
    contact1_gait.parameters["debounce_off_steps"] = "3";
    hardware_interface::InterfaceInfo contact1_gait_if;
    contact1_gait_if.name = "contact";
    contact1_gait.state_interfaces.push_back(contact1_gait_if);
    hardware_interface::InterfaceInfo contact1_gait_raw_if;
    contact1_gait_raw_if.name = "contact_raw";
    contact1_gait.state_interfaces.push_back(contact1_gait_raw_if);
    info.sensors.push_back(contact1_gait);

    return info;
  }

  std::string test_model_path_;
  hardware_interface::HardwareInfo hardware_info_;
  std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> interface_;
};

TEST_F(ContactSensorTest, ContactSensorRegistration)
{
  // Test that contact sensors are registered correctly
  // Give a small delay to ensure initialization completes
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto state_interfaces = interface_->export_state_interfaces();

  // Find contact sensor interfaces
  // Note: get_name() returns the full path "sensor_name/interface_name"
  bool found_box1_contact = false;
  bool found_box2_contact = false;
  bool found_box1_contact_raw = false;
  bool found_box2_contact_raw = false;
  bool found_box1_contact_gait = false;
  bool found_box1_contact_gait_raw = false;

  for (const auto& iface : state_interfaces)
  {
    if (iface.get_name() == "box1_contact/contact")
    {
      found_box1_contact = true;
    }
    if (iface.get_name() == "box1_contact/contact_raw")
    {
      found_box1_contact_raw = true;
    }
    if (iface.get_name() == "box2_contact/contact")
    {
      found_box2_contact = true;
    }
    if (iface.get_name() == "box2_contact/contact_raw")
    {
      found_box2_contact_raw = true;
    }
    if (iface.get_name() == "box1_contact_gait/contact")
    {
      found_box1_contact_gait = true;
    }
    if (iface.get_name() == "box1_contact_gait/contact_raw")
    {
      found_box1_contact_gait_raw = true;
    }
  }

  EXPECT_TRUE(found_box1_contact) << "box1_contact/contact interface not found";
  EXPECT_TRUE(found_box1_contact_raw) << "box1_contact/contact_raw interface not found";
  EXPECT_TRUE(found_box2_contact) << "box2_contact/contact interface not found";
  EXPECT_TRUE(found_box2_contact_raw) << "box2_contact/contact_raw interface not found";
  EXPECT_TRUE(found_box1_contact_gait) << "box1_contact_gait/contact interface not found";
  EXPECT_TRUE(found_box1_contact_gait_raw) << "box1_contact_gait/contact_raw interface not found";
}

TEST_F(ContactSensorTest, ContactDetectionWhenInContact)
{
  // Activate the interface to start simulation
  rclcpp_lifecycle::State active_state(0, "active");
  auto result = interface_->on_activate(active_state);
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Allow physics thread to start
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Wait for boxes to fall and settle on floor
  // Boxes start at z=0.1, fall time ~0.14s + settling
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Read contact values after simulation
  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));

  // Extract contact values from state interfaces
  auto state_interfaces = interface_->export_state_interfaces();
  double box1_contact_value = 0.0;
  double box2_contact_value = 0.0;
  double box1_contact_raw_value = 0.0;
  double box2_contact_raw_value = 0.0;
  bool found_box1 = false;
  bool found_box2 = false;
  bool found_box1_raw = false;
  bool found_box2_raw = false;

  for (auto& iface : state_interfaces)
  {
    if (iface.get_name() == "box1_contact/contact")
    {
      ASSERT_TRUE(iface.get_value(box1_contact_value, false));
      found_box1 = true;
    }
    if (iface.get_name() == "box1_contact/contact_raw")
    {
      ASSERT_TRUE(iface.get_value(box1_contact_raw_value, false));
      found_box1_raw = true;
    }
    if (iface.get_name() == "box2_contact/contact")
    {
      ASSERT_TRUE(iface.get_value(box2_contact_value, false));
      found_box2 = true;
    }
    if (iface.get_name() == "box2_contact/contact_raw")
    {
      ASSERT_TRUE(iface.get_value(box2_contact_raw_value, false));
      found_box2_raw = true;
    }
  }

  ASSERT_TRUE(found_box1) << "box1_contact/contact interface not found";
  ASSERT_TRUE(found_box1_raw) << "box1_contact/contact_raw interface not found";
  ASSERT_TRUE(found_box2) << "box2_contact/contact interface not found";
  ASSERT_TRUE(found_box2_raw) << "box2_contact/contact_raw interface not found";

  // In default (collision/raw) mode, contact == contact_raw.
  EXPECT_EQ(box1_contact_value, box1_contact_raw_value);
  EXPECT_EQ(box2_contact_value, box2_contact_raw_value);

  // Verify contact values are binary (0.0 or 1.0)
  EXPECT_GE(box1_contact_value, 0.0) << "box1 contact value should be valid";
  EXPECT_LE(box1_contact_value, 1.0) << "box1 contact value should be binary";
  EXPECT_GE(box2_contact_value, 0.0) << "box2 contact value should be valid";
  EXPECT_LE(box2_contact_value, 1.0) << "box2 contact value should be binary";

  // Both boxes should be in contact with floor after settling
  EXPECT_EQ(box1_contact_value, 1.0) << "box1 should be in contact with floor after settling";
  EXPECT_EQ(box2_contact_value, 1.0) << "box2 should be in contact with floor after settling";
}

TEST_F(ContactSensorTest, GaitConsumerDebounceOn)
{
  // Activate the interface to start simulation
  rclcpp_lifecycle::State active_state(0, "active");
  auto result = interface_->on_activate(active_state);
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Allow physics thread to start and objects to settle on the floor
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // With debounce_on_steps=3, and after settling, raw should be 1 immediately,
  // while filtered contact should require 3 read() calls to become 1.
  auto state_interfaces = interface_->export_state_interfaces();

  auto read_values = [&](double& contact, double& contact_raw) {
    contact = 0.0;
    contact_raw = 0.0;
    bool found_contact = false;
    bool found_raw = false;
    for (auto& iface : state_interfaces)
    {
      if (iface.get_name() == "box1_contact_gait/contact")
      {
        ASSERT_TRUE(iface.get_value(contact, false));
        found_contact = true;
      }
      if (iface.get_name() == "box1_contact_gait/contact_raw")
      {
        ASSERT_TRUE(iface.get_value(contact_raw, false));
        found_raw = true;
      }
    }
    ASSERT_TRUE(found_contact) << "box1_contact_gait/contact interface not found";
    ASSERT_TRUE(found_raw) << "box1_contact_gait/contact_raw interface not found";
  };

  // Read 1
  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  double c1, r1;
  read_values(c1, r1);
  EXPECT_EQ(r1, 1.0);
  EXPECT_EQ(c1, 0.0);

  // Read 2
  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  double c2, r2;
  read_values(c2, r2);
  EXPECT_EQ(r2, 1.0);
  EXPECT_EQ(c2, 0.0);

  // Read 3 (debounce satisfied)
  interface_->read(rclcpp::Time(), rclcpp::Duration::from_seconds(0.002));
  double c3, r3;
  read_values(c3, r3);
  EXPECT_EQ(r3, 1.0);
  EXPECT_EQ(c3, 1.0);
}

TEST_F(ContactSensorTest, ContactValueRange)
{
  // Activate the interface
  rclcpp_lifecycle::State active_state(0, "active");
  auto result = interface_->on_activate(active_state);
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Read state interfaces
  auto state_interfaces = interface_->export_state_interfaces();
  double box1_contact_value = 0.0;
  bool found_box1 = false;

  for (auto& iface : state_interfaces)
  {
    if (iface.get_name() == "box1_contact/contact")
    {
      ASSERT_TRUE(iface.get_value(box1_contact_value, false));
      found_box1 = true;
    }
  }

  ASSERT_TRUE(found_box1) << "box1_contact/contact interface not found";

  // Contact value should be either 0.0 (no contact) or 1.0 (contact)
  EXPECT_TRUE(box1_contact_value == 0.0 || box1_contact_value == 1.0)
      << "Contact value should be binary (0.0 or 1.0), got " << box1_contact_value;
}

TEST_F(ContactSensorTest, InvalidBodyNameHandling)
{
  // Test that invalid body names are handled gracefully
  hardware_interface::HardwareInfo bad_info;
  bad_info.name = "test_mujoco";
  bad_info.type = "system";
  bad_info.hardware_parameters["mujoco_model"] = test_model_path_;
  bad_info.hardware_parameters["meshdir"] = "";
  bad_info.hardware_parameters["headless"] = "true";  // Enable headless mode for CI compatibility

  // Add contact sensor with invalid body name
  hardware_interface::ComponentInfo bad_contact;
  bad_contact.name = "bad_contact";
  bad_contact.type = "sensor";
  bad_contact.parameters["mujoco_type"] = "contact";
  bad_contact.parameters["body1_name"] = "nonexistent_body";
  bad_contact.parameters["body2_name"] = "floor";
  hardware_interface::InterfaceInfo contact_if;
  contact_if.name = "contact";
  bad_contact.state_interfaces.push_back(contact_if);
  bad_info.sensors.push_back(bad_contact);

  // Create new interface with bad info
  auto bad_interface = std::make_shared<mujoco_ros2_control::MujocoSystemInterface>();
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = bad_info;

  // Should handle invalid body name gracefully (either skip or error)
  auto result = bad_interface->on_init(params);
  // Should either succeed (skipping bad sensor) or fail gracefully
  EXPECT_TRUE(result == hardware_interface::CallbackReturn::SUCCESS ||
              result == hardware_interface::CallbackReturn::ERROR);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
