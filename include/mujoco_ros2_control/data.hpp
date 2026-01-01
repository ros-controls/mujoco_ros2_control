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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "control_toolbox/pid_ros.hpp"

#include <string>
#include <vector>

namespace mujoco_ros2_control
{

/**
 * Maps to MuJoCo actuator types:
 *  - MOTOR for MuJoCo motor actuator
 *  - POSITION for MuJoCo position actuator
 *  - VELOCITY for MuJoCo velocity actuator
 *  - CUSTOM  for MuJoCo general actuator or other types
 *
 * \note the MuJoCo types are as per the MuJoCo documentation:
 * https://mujoco.readthedocs.io/en/latest/XMLreference.html#actuator
 */

enum class ActuatorType
{
  UNKNOWN,
  MOTOR,
  POSITION,
  VELOCITY,
  CUSTOM
};

/**
 * Data structure for each command/state interface.
 */
struct InterfaceData
{
  explicit InterfaceData(const std::string& name, const std::string& command_interface)
    : name_(name), command_interface_(command_interface)
  {
  }

  std::string name_;
  std::string command_interface_;
  double command_ = std::numeric_limits<double>::quiet_NaN();
  double state_ = std::numeric_limits<double>::quiet_NaN();

  // this is the "sink" that will be part of the transmission Joint/Actuator handles
  double transmission_passthrough_ = std::numeric_limits<double>::quiet_NaN();
};

/**
 * Wrapper for mujoco actuators and relevant ROS HW interface data.
 */
struct MuJoCoActuatorData
{
  std::string name = "";
  InterfaceData position_interface{ name, hardware_interface::HW_IF_POSITION };
  InterfaceData velocity_interface{ name, hardware_interface::HW_IF_VELOCITY };
  InterfaceData effort_interface{ name, hardware_interface::HW_IF_EFFORT };
  std::shared_ptr<control_toolbox::PidROS> pos_pid{ nullptr };
  std::shared_ptr<control_toolbox::PidROS> vel_pid{ nullptr };
  ActuatorType actuator_type{ ActuatorType::UNKNOWN };
  int mj_joint_type = -1;
  int mj_pos_adr = -1;
  int mj_vel_adr = -1;
  int mj_actuator_id = -1;

  // Booleans record whether or not we should be writing commands to these interfaces
  // based on if they have been claimed.
  bool is_position_control_enabled{ false };
  bool is_position_pid_control_enabled{ false };
  bool is_velocity_pid_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };
  bool has_pos_pid{ false };
  bool has_vel_pid{ false };

  void copy_state_to_transmission()
  {
    position_interface.transmission_passthrough_ = position_interface.state_;
    velocity_interface.transmission_passthrough_ = velocity_interface.state_;
    effort_interface.transmission_passthrough_ = effort_interface.state_;
  }

  void copy_command_from_transmission()
  {
    position_interface.command_ = position_interface.transmission_passthrough_;
    velocity_interface.command_ = velocity_interface.transmission_passthrough_;
    effort_interface.command_ = effort_interface.transmission_passthrough_;
  }
};

/**
 * Structure for the URDF joint data.
 */
struct URDFJointData
{
  std::string name = "";
  InterfaceData position_interface{ name, hardware_interface::HW_IF_POSITION };
  InterfaceData velocity_interface{ name, hardware_interface::HW_IF_VELOCITY };
  InterfaceData effort_interface{ name, hardware_interface::HW_IF_EFFORT };

  bool is_mimic{ false };
  int mimicked_joint_index;
  double mimic_multiplier;

  bool is_position_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };

  void copy_state_from_transmission()
  {
    position_interface.state_ = position_interface.transmission_passthrough_;
    velocity_interface.state_ = velocity_interface.transmission_passthrough_;
    effort_interface.state_ = effort_interface.transmission_passthrough_;
  }

  void copy_command_to_transmission()
  {
    position_interface.transmission_passthrough_ = position_interface.command_;
    velocity_interface.transmission_passthrough_ = velocity_interface.command_;
    effort_interface.transmission_passthrough_ = effort_interface.command_;
  }
};

template <typename T>
struct SensorData
{
  std::string name;
  T data;
  int mj_sensor_index;
};

struct FTSensorData
{
  std::string name;
  SensorData<Eigen::Vector3d> force;
  SensorData<Eigen::Vector3d> torque;
};

struct IMUSensorData
{
  std::string name;
  SensorData<Eigen::Quaternion<double>> orientation;
  SensorData<Eigen::Vector3d> angular_velocity;
  SensorData<Eigen::Vector3d> linear_acceleration;

  // These are currently unused but added to support controllers that require them.
  std::vector<double> orientation_covariance;
  std::vector<double> angular_velocity_covariance;
  std::vector<double> linear_acceleration_covariance;
};

}  // namespace mujoco_ros2_control
