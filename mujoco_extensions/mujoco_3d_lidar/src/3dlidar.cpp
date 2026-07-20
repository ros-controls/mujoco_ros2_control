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

#include "mujoco_3d_lidar/3dlidar.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

namespace mujoco::plugin::lidar
{

namespace
{

// Checks that a plugin config attribute exists.
bool CheckAttr(const std::string& input)
{
  char* end;
  std::string value = input;
  value.erase(std::remove_if(value.begin(), value.end(), isspace), value.end());
  strtod(value.c_str(), &end);
  return end == value.data() + value.size();
}

// Converts a string into a numeric vector
template <typename T>
void ReadVector(std::vector<T>& output, const std::string& input)
{
  std::stringstream ss(input);
  std::string item;
  char delim = ' ';
  while (getline(ss, item, delim))
  {
    CheckAttr(item);
    output.push_back(strtod(item.c_str(), nullptr));
  }
}

// Evenly spaced numbers over a specified interval.
void LinSpace(mjtNum lower, mjtNum upper, int n, std::vector<mjtNum>& array)
{
  if (array.size() < n)
  {
    array.resize(n);
  }
  mjtNum increment = n > 1 ? (upper - lower) / (n - 1) : 0;
  for (int i = 0; i < n; ++i)
  {
    array[i] = lower;
    lower += increment;
  }
}

}  // namespace

// Creates a Lidar instance if all config attributes are defined and
// within their allowed bounds.
Lidar* Lidar::Create(const mjModel* m, mjData* d, int instance)
{
  // resolution
  std::vector<int> resolution;
  std::string res_str = std::string(mj_getPluginConfig(m, instance, "resolution"));
  ReadVector(resolution, res_str.c_str());
  if (resolution.size() != 2)
  {
    mju_error("Both horizontal and vertical resolutions must be specified");
    return nullptr;
  }
  if (resolution[0] <= 0 || resolution[1] <= 0)
  {
    mju_error("Horizontal and vertical resolutions must be positive");
    return nullptr;
  }

  // horizontal field of view
  std::vector<mjtNum> azimuth_range;
  std::string azimuth_range_str = std::string(mj_getPluginConfig(m, instance, "azimuth_range"));
  ReadVector(azimuth_range, azimuth_range_str.c_str());
  if (azimuth_range.size() != 2)
  {
    mju_error("Both minimum and maximum azimuth angles must be specified");
    return nullptr;
  }
  if (azimuth_range[0] < -M_PI)
  {
    mju_error("`azimuth_range minimum` must be greater than or equal to -pi");
    return nullptr;
  }
  if (azimuth_range[1] > M_PI)
  {
    mju_error("`azimuth_range maximum` must be less than or equal to pi");
    return nullptr;
  }
  if (azimuth_range[0] >= azimuth_range[1])
  {
    mju_error("`azimuth_range minimum` must less than `azimuth_range maximum`");
    return nullptr;
  }

  // vertical field of view
  std::vector<mjtNum> elevation_range;
  std::string elevation_range_str = std::string(mj_getPluginConfig(m, instance, "elevation_range"));
  ReadVector(elevation_range, elevation_range_str.c_str());
  if (elevation_range.size() != 2)
  {
    if (elevation_range.size() == 0)
    {
      mju_error("Minimum elevation angle must be specified");
      return nullptr;
    }
    if (elevation_range.size() == 1)
    {
      if (resolution[1] > 1)
      {
        mju_error("When elevation resolution is greater than 1, maximum "
                  "elevation must be specified");
        return nullptr;
      }
      else
      {
        // It will need to grow
        elevation_range.push_back(elevation_range[0]);
      }
    }
  }
  if (elevation_range[0] < -M_PI)
  {
    mju_error("`elevation_range minimum` must be greater than or equal to -pi");
    return nullptr;
  }
  if (elevation_range[1] > M_PI)
  {
    mju_error("`elevation_range maximum` must be less than or equal to pi");
    return nullptr;
  }
  if ((resolution[1] > 1) && (elevation_range[0] >= elevation_range[1]))
  {
    mju_error("`elevation_range minimum` must less than `elevation_range maximum`");
    return nullptr;
  }

  std::string max_range_str = std::string(mj_getPluginConfig(m, instance, "max_range"));
  if (max_range_str.empty())
  {
    mju_error("Lidar max range must be specified");
    return nullptr;
  }
  mjtNum max_range = std::atof(max_range_str.c_str());
  if (max_range <= 0.0)
  {
    mju_error("Lidar max range must be greater than 0.0");
    return nullptr;
  }
  mjtNum min_range = 0.0;
  std::string min_range_str = std::string(mj_getPluginConfig(m, instance, "min_range"));
  if (!min_range_str.empty())
  {
    min_range = std::atof(min_range_str.c_str());
  }
  if (min_range < 0.0)
  {
    mju_error("Lidar min range must be greater than 0.0");
    return nullptr;
  }

  // Default to 1 hz
  mjtNum update_rate = 1.0;
  std::string update_rate_str = std::string(mj_getPluginConfig(m, instance, "update_rate"));
  if (!update_rate_str.empty())
  {
    update_rate = std::atof(update_rate_str.c_str());
  }
  if (update_rate < 0.0)
  {
    mju_error("Lidar update rate must be greater than 0.0");
    return nullptr;
  }

  // Default to synchronous (false)
  bool async = false;
  std::string async_str = std::string(mj_getPluginConfig(m, instance, "async"));
  if (!async_str.empty())
  {
    // Anything non-zero is true
    async = (std::atoi(async_str.c_str()) != 0);
  }

  return new Lidar(m, d, instance, resolution.data(), azimuth_range.data(), elevation_range.data(), max_range,
                   min_range, update_rate, async);
}

Lidar::Lidar(const mjModel* m, mjData* d, int instance, int resolution[2], mjtNum azimuth_range[2],
             mjtNum elevation_range[2], mjtNum max_range, mjtNum min_range, mjtNum update_rate, bool async)
  : resolution_{ resolution[0], resolution[1] }
  , max_range_(max_range)
  , min_range_(min_range)
  , update_period_(update_rate > 0.0 ? 1.0 / update_rate : 0.0)
  , last_compute_time_(-1.0)
  , async_(async)
{
  // Make sure sensor is attached to a site.
  for (int i = 0; i < m->nsensor; ++i)
  {
    if (m->sensor_type[i] == mjSENS_PLUGIN && m->sensor_plugin[i] == instance)
    {
      if (m->sensor_objtype[i] != mjOBJ_SITE)
      {
        mju_error("Lidar must be attached to a site");
      }
    }
  }

  // set up the vectors for the raycasters
  vectors_.reserve(resolution_[0] * resolution_[1] * 3);
  std::vector<mjtNum> azimuthAngles(resolution_[0]);
  std::vector<mjtNum> elevationAngles(resolution_[1]);

  if (resolution_[0] > 1)
  {
    LinSpace(azimuth_range[0], azimuth_range[1], resolution_[0], azimuthAngles);
  }
  else
  {
    azimuthAngles.push_back(0.0);
  }
  if (resolution_[1] > 1)
  {
    LinSpace(elevation_range[0], elevation_range[1], resolution_[1], elevationAngles);
  }
  else
  {
    elevationAngles.push_back(elevation_range[0]);
  }

  for (int32_t e = 0; e < resolution_[1]; ++e)
  {
    for (int32_t a = 0; a < resolution_[0]; ++a)
    {
      vectors_.push_back(std::cos(azimuthAngles[a]) * std::cos(elevationAngles[e]));  // x
      vectors_.push_back(std::sin(azimuthAngles[a]) * std::cos(elevationAngles[e]));  // y
      vectors_.push_back(std::sin(elevationAngles[e]));                               // z
    }
  }

  // Lookup sensor details based on the model
  int id;
  for (id = 0; id < m->nsensor; ++id)
  {
    if (m->sensor_type[id] == mjSENS_PLUGIN && m->sensor_plugin[id] == instance)
    {
      sensor_id_ = id;
      break;
    }
  }
  site_id_ = m->sensor_objid[sensor_id_];
  sensor_address_ = m->sensor_adr[sensor_id_];
  dimension_ = m->sensor_dim[sensor_id_];

  // Allocate storage for data
  rotated_vectors_.resize(dimension_ * 3);
  geomid_.resize(resolution_[0] * resolution_[1]);

  // Allocate async storage, noting that on extension construction mjData may still be
  // initializing. To handle that we allocate mjData lazily on the first computation
  // request.
  if (async_)
  {
    result_buf_.resize(dimension_, -1.0);
    rotated_vecs_copy_.resize(dimension_ * 3);
  }

  // Set the time point to a valid number
  d->plugin_state[m->plugin_stateadr[instance]] = 0.0;
}

Lidar::~Lidar()
{
  shutdown_.store(true);
  cv_.notify_one();
  if (worker_.joinable())
  {
    worker_.join();
  }
  if (d_copy_)
  {
    mj_deleteData(d_copy_);
  }
}

void Lidar::Reset(const mjModel* m, int instance)
{
  // No-op
}

void Lidar::Raycast(const mjModel* m, mjData* d, const mjtNum* rotated_vecs, mjtNum* output)
{
  mjtNum* site_pos = d->site_xpos + 3 * site_id_;
  mju_zero(output, dimension_);

  mjtByte* geom_group = nullptr;
  mjtByte flg_static = -1;
  int body_exclude = -1;

#if (mjVERSION_HEADER >= 3005000)
  int* geomid = nullptr;
  mj_multiRay(m, d, site_pos, rotated_vecs, geom_group, flg_static, body_exclude, geomid, output, NULL, dimension_,
              max_range_);
#elif (mjVERSION_HEADER == 340)
  mj_multiRay(m, d, site_pos, rotated_vecs, geom_group, flg_static, body_exclude, geomid_.data(), output, dimension_,
              max_range_);
#else
  mju_error("Unsupported mujoco version (%d)\n", mjVERSION_HEADER);
#endif

  for (int32_t idx = 0; idx < dimension_; ++idx)
  {
    if (output[idx] >= max_range_ || output[idx] < min_range_)
    {
      output[idx] = -1.0;
    }
  }
}

void Lidar::Compute(const mjModel* m, mjData* d, int instance)
{
  // Throttle to the update rate, if it exists and is positive
  // TODO: When we bump to later versions of mujoco we could replace this with `interval`
  if (update_period_ > 0.0 && (d->time - last_compute_time_) < update_period_)
  {
    return;
  }
  last_compute_time_ = d->time;

  // Lazy initialization handling for asynchronous processes. This is a one time cost to be sure
  // that mjData is fully initialized when we make our first copy.
  if (async_ && !worker_initialized_)
  {
    d_copy_ = mj_copyData(nullptr, m, d);

    // Async thread process that waits to be notified to do the Raycasting, will go until shutdown
    worker_ = std::thread([this, m]() {
      while (!shutdown_.load())
      {
        std::unique_lock<std::mutex> lock(data_copy_mutex_);
        cv_.wait(lock, [this] { return work_ready_ || shutdown_.load(); });

        if (shutdown_.load())
        {
          break;
        }

        work_ready_ = false;
        lock.unlock();

        Raycast(m, d_copy_, rotated_vecs_copy_.data(), result_buf_.data());

        // Notify consumers data is ready
        {
          std::lock_guard<std::mutex> lg(data_copy_mutex_);
          result_ready_ = true;
        }
      }
    });
    worker_initialized_ = true;
  }

  // Swap in completed async results as soon as they're available
  if (async_)
  {
    std::unique_lock<std::mutex> lock(data_copy_mutex_, std::try_to_lock);
    if (lock.owns_lock() && result_ready_)
    {
      std::copy(result_buf_.begin(), result_buf_.end(), d->sensordata + sensor_address_);
      d->plugin_state[m->plugin_stateadr[instance]] = result_timestamp_;
      result_ready_ = false;
    }
  }

  // Rotate ray vectors into the site's world frame
  mjtNum* site_mat = d->site_xmat + 9 * site_id_;
  for (int idx = 0; idx < dimension_; ++idx)
  {
    mjtNum vec[] = { vectors_[idx * 3], vectors_[idx * 3 + 1], vectors_[idx * 3 + 2] };
    mju_mulMatVec3(&rotated_vectors_[idx * 3], site_mat, vec);
  }

  if (async_)
  {
    // If running asynchronously, we snapshot the data and signal the worker thread to trigger
    // the raycasting. However, it is possible that the worker is still doing a compute. If that
    // is the case we skip this cycle.
    std::unique_lock<std::mutex> lock(data_copy_mutex_);
    if (work_ready_)
    {
      return;
    }

    // Copy the snapshot
    mj_copyData(d_copy_, m, d);
    rotated_vecs_copy_ = rotated_vectors_;
    result_timestamp_ = d->time;

    // Signal the worker thread
    work_ready_ = true;
    lock.unlock();
    cv_.notify_one();
  }
  else
  {
    // Otherwise we compute and copy directly into the data container
    mjtNum* sensordata = d->sensordata + sensor_address_;
    Raycast(m, d, rotated_vectors_.data(), sensordata);

    // Essentially a time stamp that can be accessed from consumers to know when the last
    // reading was taken
    d->plugin_state[m->plugin_stateadr[instance]] = last_compute_time_;
  }
}

void Lidar::Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance)
{
  if (!opt->flags[mjVIS_RANGEFINDER])
  {
    return;
  }

  // Get site frame.
  mjtNum* site_pos = d->site_xpos + 3 * site_id_;

  const mjtNum* dataptr = d->sensordata + sensor_address_;

  // get point and draw line if dist is valid
  mjtNum point[3] = { 0 };
  for (int idx = 0; idx < dimension_; ++idx)
  {
    mjtNum dist = dataptr[idx];
    if (dist >= 0 && scn->ngeom < scn->maxgeom)
    {
      for (uint8_t i = 0; i < 3; ++i)
      {
        point[i] = site_pos[i] + rotated_vectors_[idx * 3 + i] * dist;
      }

      mjvGeom* thisgeom = scn->geoms + scn->ngeom;
      mjv_initGeom(thisgeom, mjGEOM_LINE, nullptr, nullptr, nullptr, m->vis.rgba.rangefinder);

      mjv_connector(thisgeom, mjGEOM_LINE, 1, site_pos, point);
      thisgeom->objtype = mjOBJ_UNKNOWN;
      thisgeom->objid = sensor_id_;
      thisgeom->category = mjCAT_DECOR;
      thisgeom->segid = scn->ngeom;
      scn->ngeom++;
    }
  }
}

void Lidar::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "mujoco.plugin.lidar";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;

  // Parameterizable attributes
  const char* attributes[] = { "resolution",  "azimuth_range", "elevation_range", "max_range", "min_range",
                               "update_rate", "async" };
  plugin.nattribute = sizeof(attributes) / sizeof(attributes[0]);
  plugin.attributes = attributes;

  // Maintain last updated state.
  plugin.nstate = +[](const mjModel* m, int instance) { return 1; };

  // Sensor dimension = resolution[0] * resolution[1]
  plugin.nsensordata = +[](const mjModel* m, int instance, int sensor_id) {
    std::vector<int> size;
    std::string res_str = std::string(mj_getPluginConfig(m, instance, "resolution"));
    ReadVector(size, res_str.c_str());
    return size[0] * size[1];
  };

  // Can only run after forces have been computed.
  plugin.needstage = mjSTAGE_ACC;

  // Initialization callback.
  plugin.init = +[](const mjModel* m, mjData* d, int instance) {
    auto* Lidar = Lidar::Create(m, d, instance);
    if (!Lidar)
    {
      return -1;
    }
    d->plugin_data[instance] = reinterpret_cast<uintptr_t>(Lidar);
    return 0;
  };

  // Destruction callback.
  plugin.destroy = +[](mjData* d, int instance) {
    delete reinterpret_cast<Lidar*>(d->plugin_data[instance]);
    d->plugin_data[instance] = 0;
  };

  // Reset callback.
  plugin.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance) {
    auto* Lidar = reinterpret_cast<class Lidar*>(plugin_data);
    Lidar->Reset(m, instance);
  };

  // Compute callback.
  plugin.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
    auto* Lidar = reinterpret_cast<class Lidar*>(d->plugin_data[instance]);
    Lidar->Compute(m, d, instance);
  };

  // Visualization callback.
  plugin.visualize = +[](const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance) {
    auto* Lidar = reinterpret_cast<class Lidar*>(d->plugin_data[instance]);
    Lidar->Visualize(m, d, opt, scn, instance);
  };

  // Register the plugin.
  mjp_registerPlugin(&plugin);
}

}  // namespace mujoco::plugin::lidar
