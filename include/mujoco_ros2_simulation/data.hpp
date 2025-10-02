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

#include <string>
#include <vector>

namespace mujoco_ros2_simulation
{

/**
 * Wrapper for mujoco actuators and relevant ROS HW interface data.
 */
struct JointState
{
  std::string name;
  double position;
  double velocity;
  double effort;
  double position_command;
  double velocity_command;
  double effort_command;
  bool is_mimic{ false };
  int mimicked_joint_index;
  double mimic_multiplier;
  int mj_joint_type;
  int mj_pos_adr;
  int mj_vel_adr;
  int mj_actuator_id;

  // Booleans record whether or not we should be writing commands to these interfaces
  // based on if they have been claimed.
  bool is_position_control_enabled{ false };
  bool is_velocity_control_enabled{ false };
  bool is_effort_control_enabled{ false };
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

}  // namespace mujoco_ros2_simulation
