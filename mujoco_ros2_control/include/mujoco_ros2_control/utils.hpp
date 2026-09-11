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

#include "mujoco_ros2_control/data.hpp"

#include <cmath>
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
 * @brief Returns the value of a sensor-level `<param>` (from the sensor's own ComponentInfo), or a default
 * value if the sensor has no such parameter set.
 */
inline std::string get_sensor_parameter_or(const hardware_interface::ComponentInfo& sensor, const std::string& key,
                                           const std::string& default_value)
{
  if (auto it = sensor.parameters.find(key); it != sensor.parameters.end())
  {
    return it->second;
  }
  return default_value;
}

/**
 * @brief Reads the sensor's `noise_distribution` parameter ("gaussian" (default) or "uniform").
 * Anything other than exactly "uniform" (including an absent parameter) is treated as Gaussian.
 */
inline NoiseDistribution get_noise_distribution(const hardware_interface::ComponentInfo& sensor)
{
  return get_sensor_parameter_or(sensor, "noise_distribution", "gaussian") == "uniform" ?
             NoiseDistribution::kUniform :
             NoiseDistribution::kGaussian;
}

namespace detail
{

/**
 * @brief Draws a single zero-mean noise sample with the given standard deviation and shape.
 *
 * Uniform noise is drawn from `[-stddev*sqrt(3), stddev*sqrt(3)]` (the range of a uniform distribution
 * whose own standard deviation is `stddev`), so switching a sensor's `noise_distribution` doesn't change
 * the magnitude implied by its MJCF `noise` value.
 */
inline double sample_noise(double stddev, NoiseDistribution distribution, std::mt19937& rng)
{
  if (distribution == NoiseDistribution::kUniform)
  {
    const double bound = stddev * std::sqrt(3.0);
    std::uniform_real_distribution<double> dist(-bound, bound);
    return dist(rng);
  }
  std::normal_distribution<double> dist(0.0, stddev);
  return dist(rng);
}

}  // namespace detail

/**
 * @brief Adds zero-mean noise, independently sampled per axis, to a 3D vector in place.
 * No-op if `stddev` is not positive, so a disabled (default) sensor pays no sampling cost.
 */
inline void add_sensor_noise(Eigen::Vector3d& value, double stddev, NoiseDistribution distribution,
                              std::mt19937& rng)
{
  if (stddev <= 0.0)
  {
    return;
  }
  value.x() += detail::sample_noise(stddev, distribution, rng);
  value.y() += detail::sample_noise(stddev, distribution, rng);
  value.z() += detail::sample_noise(stddev, distribution, rng);
}

/**
 * @brief Adds zero-mean noise to a quaternion's coefficients, then renormalizes.
 *
 * This is a small-angle approximation of orientation noise: accurate for the small stddev values noise
 * configuration is expected to use, but not a proper noise model on SO(3) for large values.
 * No-op if `stddev` is not positive.
 */
inline void add_sensor_noise(Eigen::Quaterniond& value, double stddev, NoiseDistribution distribution,
                              std::mt19937& rng)
{
  if (stddev <= 0.0)
  {
    return;
  }
  value.coeffs() += Eigen::Vector4d(detail::sample_noise(stddev, distribution, rng),
                                    detail::sample_noise(stddev, distribution, rng),
                                    detail::sample_noise(stddev, distribution, rng),
                                    detail::sample_noise(stddev, distribution, rng));
  value.normalize();
}

}  // namespace mujoco_ros2_control
