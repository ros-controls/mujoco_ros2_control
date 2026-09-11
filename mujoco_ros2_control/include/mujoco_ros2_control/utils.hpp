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
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control/data.hpp"

#include <cassert>
#include <cmath>
#include <random>
#include <string>

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
 * A value that is neither (including a typo) logs a warning and is treated as Gaussian; an absent
 * parameter is treated as Gaussian silently.
 */
inline NoiseDistribution get_noise_distribution(const hardware_interface::ComponentInfo& sensor)
{
  const std::string value = get_sensor_parameter_or(sensor, "noise_distribution", "gaussian");
  if (value != "gaussian" && value != "uniform")
  {
    RCLCPP_WARN(rclcpp::get_logger("mujoco_ros2_control"),
                "Only noise distributions of 'gaussian' or 'uniform' are allowed, but you selected '%s'. "
                "Defaulting to 'gaussian'.",
                value.c_str());
  }
  return value == "uniform" ? NoiseDistribution::kUniform : NoiseDistribution::kGaussian;
}

/**
 * @brief Sets a flat, row-major 3x3 covariance's diagonal to `stddev * stddev`, leaving off-diagonal terms
 * untouched (0 for a freshly-`resize()`d vector, matching the "independent per-axis noise" assumption).
 */
inline void set_diagonal_covariance(std::vector<double>& covariance, double stddev)
{
  assert(covariance.size() == 9 && "covariance must be a flat 3x3 (9-element) matrix");
  covariance[0] = covariance[4] = covariance[8] = stddev * stddev;
}

namespace detail
{

/**
 * @brief Fills an N-vector with independent zero-mean noise samples of the given standard deviation and
 * shape, constructing the underlying distribution once and drawing all N samples from it.
 *
 * Uniform noise is drawn from `[-stddev*sqrt(3), stddev*sqrt(3)]` (the range of a uniform distribution
 * whose own standard deviation is `stddev`), so switching a sensor's `noise_distribution` doesn't change
 * the magnitude implied by its MJCF `noise` value.
 */
template <int N>
inline Eigen::Matrix<double, N, 1> sample_noise(double stddev, NoiseDistribution distribution, std::mt19937& rng)
{
  Eigen::Matrix<double, N, 1> samples;
  if (distribution == NoiseDistribution::kUniform)
  {
    const double bound = stddev * std::sqrt(3.0);
    std::uniform_real_distribution<double> dist(-bound, bound);
    for (int i = 0; i < N; ++i)
    {
      samples[i] = dist(rng);
    }
  }
  else
  {
    std::normal_distribution<double> dist(0.0, stddev);
    for (int i = 0; i < N; ++i)
    {
      samples[i] = dist(rng);
    }
  }
  return samples;
}

}  // namespace detail

/**
 * @brief Adds zero-mean noise, independently sampled per axis, to a 3D vector in place.
 * No-op if `stddev` is not positive, so a disabled (default) sensor pays no sampling cost.
 */
inline void add_sensor_noise(Eigen::Vector3d& value, double stddev, NoiseDistribution distribution, std::mt19937& rng)
{
  if (stddev <= 0.0)
  {
    return;
  }
  value += detail::sample_noise<3>(stddev, distribution, rng);
}

/**
 * @brief Adds zero-mean noise to a quaternion's coefficients, then renormalizes.
 *
 * This is a small-angle approximation of orientation noise: accurate for the small stddev values noise
 * configuration is expected to use, but not a proper noise model on SO(3) for large values.
 * No-op if `stddev` is not positive.
 */
inline void add_sensor_noise(Eigen::Quaterniond& value, double stddev, NoiseDistribution distribution, std::mt19937& rng)
{
  if (stddev <= 0.0)
  {
    return;
  }
  value.coeffs() += detail::sample_noise<4>(stddev, distribution, rng);
  value.normalize();
}

/**
 * @brief Convenience overloads taking a sensor's bundled NoiseState instead of a separate
 * distribution/rng pair.
 */
inline void add_sensor_noise(Eigen::Vector3d& value, double stddev, NoiseState& noise)
{
  add_sensor_noise(value, stddev, noise.distribution, noise.rng);
}

inline void add_sensor_noise(Eigen::Quaterniond& value, double stddev, NoiseState& noise)
{
  add_sensor_noise(value, stddev, noise.distribution, noise.rng);
}

}  // namespace mujoco_ros2_control
