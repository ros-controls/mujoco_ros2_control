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

#ifndef MUJOCO_PLUGIN_SENSOR_LIDAR_H_
#define MUJOCO_PLUGIN_SENSOR_LIDAR_H_

#include <optional>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::lidar
{

// The sensor has a number of parameters:
//  1. (int) Horizontal resolution.
//  2. (int) Vertical resolution.
//  3. (double) Horizontal field-of-view (fov_x), in degrees.
//  4. (double) Vertical field-of-view (fov_y), in degrees.
//  5. (double) Maximum range, in m.
//  6. (double) Minimum range, in m (optional, defaults to 0).
//  7. (double) Rate at which to update sensor readings, in Hz (optional, defaults to 0).

class Lidar
{
public:
  static Lidar* Create(const mjModel* m, mjData* d, int instance);
  Lidar(Lidar&&) = default;
  ~Lidar() = default;

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);

  static void RegisterPlugin();

private:
  Lidar(const mjModel* m, mjData* d, int instance, int resolution[2], mjtNum azimuth_range[2],
        mjtNum elevation_range[2], mjtNum max_range, mjtNum min_range, mjtNum update_rate);
  std::vector<mjtNum> vectors_;
  std::vector<mjtNum> rotated_vectors_;
  std::vector<int> geomid_;

  int sensor_id_;             // sensor id of the sensor in the model
  int site_id_;               // site id of the sensor in the model
  int sensor_address_;        // Address in the model based on sensor id
  int dimension_;             // Dimension of the sensor
  int resolution_[2];         // horizontal and vertical resolution
  mjtNum fov_[2];             // horizontal and vertical field of view, in degrees
  mjtNum max_range_;          // max range of lidar
  mjtNum min_range_;          // min range of lidar
  mjtNum update_period_;      // Update period to run
  mjtNum last_compute_time_;  // Sim time of the last reading
};

}  // namespace mujoco::plugin::lidar

#endif  // MUJOCO_PLUGIN_SENSOR_LIDAR_H_
