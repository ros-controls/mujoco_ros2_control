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

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

namespace mujoco::plugin::lidar
{

/**
 * @brief Lidar sensor implementation based on `mj_multiRay`.
 *
 * The sensor supports synchronous and asynchronous operations, as lidar computations can get
 * quite heavy with many rays. In either mode, raycasting computations are done at the requested
 * `update_period_`. In synchronous operations, the computation will block `mj_step` and update
 * the sensor's data immediately.
 *
 * In asynchronous operation, a copy of the data is taken and a background process will handle
 * the expensive computations, then write data to the sensor's data block on the next update.
 * This means that the readings may be 1 update rate timestep behind reality, or possibly longer
 * depending on whether or not the computation can be completed in one cycle. However, the included
 * timestamp will contain the simulation time that the data was copied from, so that consumers
 * can check how delayed sensor readings are.
 *
 * Note that copying mjData is expensive, so it is up to consumers of this plugin to decide if
 * the timing tradeoff is worth it.
 */
class Lidar
{
public:
  static Lidar* Create(const mjModel* m, mjData* d, int instance);
  Lidar(Lidar&&) = default;
  ~Lidar();

  void Reset(const mjModel* m, int instance);
  void Compute(const mjModel* m, mjData* d, int instance);
  void Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);

  static void RegisterPlugin();

private:
  Lidar(const mjModel* m, mjData* d, int instance, int resolution[2], mjtNum azimuth_range[2],
        mjtNum elevation_range[2], mjtNum max_range, mjtNum min_range, mjtNum update_rate, bool async);

  void Raycast(const mjModel* m, mjData* d, const mjtNum* rotated_vecs, mjtNum* output);

  void ProcessRaycastAsync(const mjModel* m);

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
  bool async_;                // Whether or not to do raycasting in a background thread

  // Storage containers for asynchronous raycasting
  mjData* d_copy_{ nullptr };              // mjData snapshot for the worker thread
  std::vector<mjtNum> result_buf_;         // worker writes finished ranges here
  std::vector<mjtNum> rotated_vecs_copy_;  // snapshot of rotated vectors for worker
  mjtNum result_timestamp_{ -1.0 };        // sim-time the result corresponds to

  // The following are all used for asynchronous processing thread control.
  std::mutex data_copy_mutex_;
  std::condition_variable cv_;
  std::thread worker_;
  bool worker_initialized_{ false };
  bool work_ready_{ false };
  bool result_ready_{ false };
  std::atomic<bool> shutdown_{ false };
};

}  // namespace mujoco::plugin::lidar

#endif  // MUJOCO_PLUGIN_SENSOR_LIDAR_H_