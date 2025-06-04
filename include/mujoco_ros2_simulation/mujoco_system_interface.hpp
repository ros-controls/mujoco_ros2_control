// Copyright 2025 NASA Johnson Space Center
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

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <mujoco/mujoco.h>

// Pull in the Simulate class and PhysicsThread/RenderLoop declarations:
#include "simulate.h"     // must be on your include path, handled by CMake
#include "glfw_adapter.h" // for mj::GlfwAdapter

namespace mujoco_ros2_simulation
{
  class MujocoSystemInterface : public hardware_interface::SystemInterface
  {
  public:

    MujocoSystemInterface();
    ~MujocoSystemInterface() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // System information
    hardware_interface::HardwareInfo system_info_;
    std::string model_path_;

    // MuJoCo data pointers
    mjModel *model_{nullptr};
    mjData *data_{nullptr};

    // Primary simulate object
    std::unique_ptr<mujoco::Simulate> sim_;

    // Threads for rendering physics and the UI window
    std::thread physics_thread_;
    std::thread ui_thread_;

    // Mutex used inside simulate.h for protecting model/data, we keep a reference here
    // to protect access to shared data.
    std::recursive_mutex *sim_mutex_{nullptr};

    // Joints and actuator data
    // TODO: Break these and maybe recreate jointstate objects
    size_t n_joints_{0};
    std::vector<std::string> joint_names_;

    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_commands_; // Currently just position commands

    // And then put these into them, since a bunch of vectors is a little nerve wracking
    std::vector<int> muj_joint_id_;
    std::vector<int> muj_actuator_id_;
  };

} // end namespace
