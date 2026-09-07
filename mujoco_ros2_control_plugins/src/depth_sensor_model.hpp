/**
 * Copyright (c) 2026 PAL Robotics S.L.
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

#include <algorithm>
#include <cstdint>
#include <limits>
#include <random>

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Turns MuJoCo's exact geometric depth into a reading a stereo depth camera could return.
 *
 * MuJoCo reports the true distance at every pixel: no noise, no invalid returns, and values
 * below the sensor's real Min-Z. Anything built on that stream is tuned against a sensor that
 * does not exist, and the discrepancy only shows up on hardware. The defaults describe an
 * Intel RealSense D435i; every one of them is a parameter because the right value depends on
 * the device being simulated.
 *
 * Kept separate from the plugin so the arithmetic can be unit tested without an OpenGL context.
 */
class DepthSensorModel
{
public:
  float min_range{ 0.28f };        ///< [m] Min-Z; below this the sensor returns nothing.
  float max_range{ 3.0f };         ///< [m] beyond the ideal range.
  float stereo_baseline{ 0.05f };  ///< [m] separation of the IR pair.
  float subpixel_error{ 0.15f };   ///< [px] disparity matching error; about 1% at 2 m.
  double dropout_fraction{ 0.02 }; ///< Fraction of pixels that return nothing.

  /// Seeded deterministically so a recording can be reproduced; reseed for independent runs.
  explicit DepthSensorModel(std::uint32_t seed = 42u) : generator_(seed)
  {
  }

  /// Restarts the noise sequence. The distributions are reset too: normal_distribution
  /// caches a second Box-Muller value, so reseeding the generator alone would not.
  void reseed(std::uint32_t seed)
  {
    generator_.seed(seed);
    noise_.reset();
    dropout_.reset();
  }

  /**
   * @brief Standard deviation of the triangulation error at @p depth.
   *
   * Stereo depth error grows with the SQUARE of range, because depth is inversely
   * proportional to disparity:
   *
   *     sigma(Z) = Z^2 * subpixel / (focal_px * baseline)
   *
   * Derived from the geometry rather than fixed as a percentage, so it stays correct if the
   * resolution or the field of view changes. Pass the very focal length published in
   * camera_info, so the noise matches the geometry the consumer will reproject with.
   *
   * At the 0.15 px default with a 50 mm baseline:
   *
   *            fx 617 (colour/depth)      fx 433 (infra)
   *   0.5 m       1.2 mm  (0.24%)          1.7 mm  (0.35%)
   *   1.0 m       4.9 mm  (0.49%)          6.9 mm  (0.69%)
   *   2.0 m      19.4 mm  (0.97%)         27.7 mm  (1.39%)
   *   3.0 m      43.7 mm  (1.46%)         62.4 mm  (2.08%)
   *
   * That is about 1% at 2 m, the figure typically reported for a D435i rather than the
   * datasheet's "<2% at 2 m" worst case. Raise subpixel_error to roughly 0.3 px to sit on
   * that bound instead; the parameter is the honest place to say how pessimistic to be.
   */
  [[nodiscard]] float stddevAt(float depth, float focal_px) const
  {
    return depth * depth * subpixel_error / std::max(1e-6f, focal_px * stereo_baseline);
  }

  /**
   * @brief Apply the model to one exact depth sample.
   *
   * @return NaN outside the usable range and for a dropped pixel -- the ROS convention for
   *         "no data" in a 32FC1 image -- otherwise the sample perturbed by triangulation
   *         error. Returning a number instead of NaN is what lets a consumer trust geometry
   *         the sensor could never have supplied.
   */
  float apply(float true_depth, float focal_px)
  {
    // Written as !(x >= min) so that a NaN input stays NaN rather than passing the test.
    if (!(true_depth >= min_range) || true_depth > max_range)
    {
      return std::numeric_limits<float>::quiet_NaN();
    }

    // Holes. Real depth drops out on low-texture, specular and occluded surfaces, none of
    // which MuJoCo knows about, so this is a blunt stand-in for their FREQUENCY rather than
    // their spatial structure: real dropouts come in patches, these do not.
    if (dropout_fraction > 0.0 && dropout_(generator_) < dropout_fraction)
    {
      return std::numeric_limits<float>::quiet_NaN();
    }

    const float noisy = true_depth + stddevAt(true_depth, focal_px) * static_cast<float>(noise_(generator_));

    // Noise can push a sample out of range; it is still a reading the sensor would not return.
    if (!(noisy >= min_range) || noisy > max_range)
    {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return noisy;
  }

private:
  std::mt19937 generator_;
  std::normal_distribution<double> noise_{ 0.0, 1.0 };
  std::uniform_real_distribution<double> dropout_{ 0.0, 1.0 };
};

}  // namespace mujoco_ros2_control_plugins
