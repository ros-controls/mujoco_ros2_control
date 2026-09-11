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
#include <hardware_interface/hardware_info.hpp>

#include <random>

namespace mujoco_ros2_control
{

/**
 * @brief Returns the sensor's component info for the provided sensor name, if it exists.
 */
inline std::optional<hardware_interface::ComponentInfo>
get_sensor_from_info(const hardware_interface::HardwareInfo& hardware_info, const std::string& name)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    const auto& sensor = hardware_info.sensors.at(sensor_index);
    if (hardware_info.sensors.at(sensor_index).name == name)
    {
      return sensor;
    }
  }
  return std::nullopt;
}

/**
 * @brief Adds zero-mean Gaussian noise, independently sampled per axis, to a 3D vector in place.
 * No-op if `stddev` is not positive, so a disabled (default) sensor pays no sampling cost.
 */
inline void add_gaussian_noise(Eigen::Vector3d& value, double stddev, std::mt19937& rng)
{
  if (stddev <= 0.0)
  {
    return;
  }
  std::normal_distribution<double> dist(0.0, stddev);
  value.x() += dist(rng);
  value.y() += dist(rng);
  value.z() += dist(rng);
}

/**
 * @brief Adds zero-mean Gaussian noise to a quaternion's coefficients, then renormalizes.
 *
 * This is a small-angle approximation of orientation noise: accurate for the small stddev values noise
 * configuration is expected to use, but not a proper noise model on SO(3) for large values.
 * No-op if `stddev` is not positive.
 */
inline void add_gaussian_noise(Eigen::Quaterniond& value, double stddev, std::mt19937& rng)
{
  if (stddev <= 0.0)
  {
    return;
  }
  std::normal_distribution<double> dist(0.0, stddev);
  value.coeffs() += Eigen::Vector4d(dist(rng), dist(rng), dist(rng), dist(rng));
  value.normalize();
}

}  // namespace mujoco_ros2_control
