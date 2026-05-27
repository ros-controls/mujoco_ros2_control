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
    mju_error("`azimuth_range minimum` must less than 'azimuth_range maximum");
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
      mju_error("Mininimum elevation angle must be specified");
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
        elevation_range[1] = elevation_range[0];
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
    mju_error("`elevation_range minimum` must less than 'elevation_range maximum");
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
  mjtNum update_rate = 0.0;
  std::string update_rate_str = std::string(mj_getPluginConfig(m, instance, "update_rate"));
  if (!update_rate_str.empty())
  {
    update_rate = std::atof(update_rate_str.c_str());
  }

  // For debugging
  // printf("     resolution = %d, %d\n", resolution[0], resolution[1]);
  // printf("  azimuth_range = %.3lf - %.3lf\n", azimuth_range[0], azimuth_range[1]);
  // printf("elevation_range = %.3lf - %.3lf\n", elevation_range[0], elevation_range[1]);
  // printf("      max_range = %.3lf\n", max_range);
  // printf("      min_range = %.3lf\n", min_range);
  // printf("    update_rate = %.3lf\n", update_rate);

  return new Lidar(m, d, instance, resolution.data(), azimuth_range.data(), elevation_range.data(), max_range,
                   min_range, update_rate);
}

Lidar::Lidar(const mjModel* m, mjData* d, int instance, int resolution[2], mjtNum azimuth_range[2],
             mjtNum elevation_range[2], mjtNum max_range, mjtNum min_range, mjtNum update_rate)
  : resolution_{ resolution[0], resolution[1] }
  , max_range_(max_range)
  , min_range_(min_range)
  , update_period_(update_rate > 0.0 ? 1.0 / update_rate : 0.0)
  , last_compute_time_(-1.0)
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
  std::vector<mjtNum> azmuthAngles(resolution_[0]);
  std::vector<mjtNum> elevationAngles(resolution_[1]);

  if (resolution_[0] > 1)
  {
    LinSpace(azimuth_range[0], azimuth_range[1], resolution_[0], azmuthAngles);
  }
  else
  {
    azmuthAngles.push_back(0.0);
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
      // x
      vectors_.push_back(std::cos(azmuthAngles[a]) * std::cos(elevationAngles[e]));
      // y
      vectors_.push_back(std::sin(azmuthAngles[a]) * std::cos(elevationAngles[e]));
      // z
      vectors_.push_back(std::sin(elevationAngles[e]));
    }
  }
}

void Lidar::Reset(const mjModel* m, int instance)
{
}

void Lidar::Compute(const mjModel* m, mjData* d, int instance)
{
  // Throttle to the update rate, if it exists and is positive
  if (update_period_ > 0.0 && (d->time - last_compute_time_) < update_period_)
  {
    return;
  }
  last_compute_time_ = d->time;

  // Essentially a time stamp that can be accessed from consumers to know when the last
  // reading was taken
  d->plugin_state[m->plugin_stateadr[instance]] = last_compute_time_;

  mjtByte* geom_group = nullptr;
  mjtByte flg_static = -1;
  int body_exclude = -1;

  // Get site id.
  int32_t id;
  for (id = 0; id < m->nsensor; ++id)
  {
    if (m->sensor_type[id] == mjSENS_PLUGIN && m->sensor_plugin[id] == instance)
    {
      break;
    }
  }
  int site_id = m->sensor_objid[id];

  // Get site frame.
  mjtNum* site_pos = d->site_xpos + 3 * site_id;
  mjtNum* site_mat = d->site_xmat + 9 * site_id;

  int adr = m->sensor_adr[id];
  int dim = m->sensor_dim[id];

  mj_markStack(d);

  // Clear sensordata and distance matrix.
  mjtNum* sensordata = d->sensordata + adr;
  mju_zero(sensordata, dim);

  rotated_vectors_.clear();
  rotated_vectors_.reserve(dim * 3);
  for (int idx = 0; idx < dim; ++idx)
  {
    mjtNum vec[] = { vectors_[idx * 3], vectors_[idx * 3 + 1], vectors_[idx * 3 + 2] };
    mjtNum res[] = { 0.0, 0.0, 0.0 };
    mju_mulMatVec3(res, site_mat, vec);
    rotated_vectors_.push_back(res[0]);
    rotated_vectors_.push_back(res[1]);
    rotated_vectors_.push_back(res[2]);
  }

#if (mjVERSION_HEADER == 3005000)
  int* geomid = nullptr;
  mj_multiRay(m, d, site_pos, rotated_vectors_.data(), geom_group, flg_static, body_exclude, geomid, sensordata, NULL,
              resolution_[0] * resolution_[1], max_range_);
#elif (mjVERSION_HEADER == 340)
  std::vector<int> geomid(resolution_[0] * resolution_[1]);
  mj_multiRay(m, d, site_pos, rotated_vectors_.data(), geom_group, flg_static, body_exclude, geomid.data(), sensordata,
              resolution_[0] * resolution_[1], max_range_);
#else
  mju_error("Unsupported mujoco version (%d)\n", mjVERSION_HEADER);
#endif

  for (int32_t idx = 0; idx < resolution_[0] * resolution_[1]; ++idx)
  {
    if (sensordata[idx] >= max_range_ || sensordata[idx] < min_range_)
    {
      sensordata[idx] = -1.0;
    }
  }
  mj_freeStack(d);
}

void Lidar::Visualize(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance)
{
  if (!opt->flags[mjVIS_RANGEFINDER])
  {
    return;
  }

  // Get sensor id.
  int id;
  for (id = 0; id < m->nsensor; ++id)
  {
    if (m->sensor_type[id] == mjSENS_PLUGIN && m->sensor_plugin[id] == instance)
    {
      break;
    }
  }

  // Get sensor data.
  if (id < m->nsensor)
  {
    int site_id = m->sensor_objid[id];

    // Get site frame.
    mjtNum* site_pos = d->site_xpos + 3 * site_id;

    int adr = m->sensor_adr[id];
    int dim = m->sensor_dim[id];

    const mjtNum* dataptr = d->sensordata + adr;

    // get point and draw line if dist is valid
    mjtNum point[3] = { 0 };
    if (rotated_vectors_.size() == dim * 3)
    {
      for (int idx = 0; idx < dim; ++idx)
      {
        mjtNum dist = dataptr[idx];
        if (dist >= 0 && scn->ngeom < scn->maxgeom)
        {
          point[0] = site_pos[0] + rotated_vectors_[idx * 3] * dist;
          point[1] = site_pos[1] + rotated_vectors_[idx * 3 + 1] * dist;
          point[2] = site_pos[2] + rotated_vectors_[idx * 3 + 2] * dist;

          mjvGeom* thisgeom = scn->geoms + scn->ngeom;
          mjv_initGeom(thisgeom, mjGEOM_LINE, nullptr, nullptr, nullptr, m->vis.rgba.rangefinder);

          mjv_connector(thisgeom, mjGEOM_LINE, 1, site_pos, point);
          thisgeom->objtype = mjOBJ_UNKNOWN;
          thisgeom->objid = id;
          thisgeom->category = mjCAT_DECOR;
          thisgeom->segid = scn->ngeom;
          scn->ngeom++;
        }
      }
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
  const char* attributes[] = {
    "resolution", "azimuth_range", "elevation_range", "max_range", "min_range", "update_rate"
  };
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
