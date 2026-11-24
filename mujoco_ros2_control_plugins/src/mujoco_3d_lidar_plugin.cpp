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

#include "mujoco_3d_lidar_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mujoco_ros2_control_plugins
{

namespace
{

// Evenly spaced numbers over a specified interval.
void lin_space(float lower, float upper, int n, std::vector<float>& array)
{
  if (static_cast<int>(array.size()) < n)
  {
    array.resize(n);
  }
  float increment = n > 1 ? (upper - lower) / static_cast<float>(n - 1) : 0.0F;
  for (int i = 0; i < n; ++i)
  {
    array[i] = lower;
    lower += increment;
  }
}

void compute_vectors(Lidar3dConfig& lidar)
{
  lidar.vectors.clear();
  lidar.vectors.reserve(lidar.resolution[0] * lidar.resolution[1]);
  std::vector<float> azmuthAngles(lidar.resolution[0]);
  std::vector<float> elevationAngles(lidar.resolution[1]);

  if (lidar.resolution[0] > 1)
  {
    lin_space(static_cast<float>(lidar.azimuth_range[0]), static_cast<float>(lidar.azimuth_range[1]),
              lidar.resolution[0], azmuthAngles);
  }
  else
  {
    azmuthAngles.push_back(0.0);
  }
  if (lidar.resolution[1] > 1)
  {
    lin_space(static_cast<float>(lidar.elevation_range[0]), static_cast<float>(lidar.elevation_range[1]),
              lidar.resolution[1], elevationAngles);
  }
  else
  {
    elevationAngles.push_back(static_cast<float>(lidar.elevation_range[0]));
  }

  for (int32_t e = 0; e < lidar.resolution[1]; ++e)
  {
    for (int32_t a = 0; a < lidar.resolution[0]; ++a)
    {
      geometry_msgs::msg::Vector3 vec;
      vec.x = (std::cos(azmuthAngles[a]) * std::cos(elevationAngles[e]));
      vec.y = (std::sin(azmuthAngles[a]) * std::cos(elevationAngles[e]));
      vec.z = (std::sin(elevationAngles[e]));
      lidar.vectors.push_back(vec);
    }
  }
}

bool is_big_endian(void)
{
  union
  {
    uint32_t i;
    char c[4];
  } bint = { 0x01020304 };

  return bint.c[0] == 1;
}

// Breaks a string into a generic T vector
template <typename T>
void read_vector(std::vector<T>& output, const std::string& input)
{
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, ' '))
  {
    if (!item.empty())
    {
      output.push_back(static_cast<T>(std::strtod(item.c_str(), nullptr)));
    }
  }
}

// Sets the fields of a cloud in order, each at an offset aligned to its size, and returns the point step,
// a multiple of the largest field size
uint32_t set_aligned_fields(sensor_msgs::msg::PointCloud2& cloud,
                            const std::vector<std::pair<std::string, uint8_t>>& fields)
{
  auto size_of = [](uint8_t datatype) -> uint32_t {
    switch (datatype)
    {
      case sensor_msgs::msg::PointField::INT8:
      case sensor_msgs::msg::PointField::UINT8:
        return 1;
      case sensor_msgs::msg::PointField::INT16:
      case sensor_msgs::msg::PointField::UINT16:
        return 2;
      case sensor_msgs::msg::PointField::FLOAT64:
        return 8;
      default:
        return 4;
    }
  };
  cloud.fields.clear();
  uint32_t offset = 0;
  uint32_t max_size = 1;
  for (const auto& [name, datatype] : fields)
  {
    const uint32_t size = size_of(datatype);
    offset = (offset + size - 1) / size * size;
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    cloud.fields.push_back(field);
    offset += size;
    max_size = std::max(max_size, size);
  }
  return (offset + max_size - 1) / max_size * max_size;
}

// Name of a point layout, as set in the point_fields parameter
const char* point_fields_name(PointLayout layout)
{
  switch (layout)
  {
    case PointLayout::ROBOSENSE:
      return "robosense";
    case PointLayout::VELODYNE:
      return "velodyne";
    case PointLayout::OUSTER:
      return "ouster";
    default:
      return "xyz";
  }
}

}  // namespace

bool Mujoco3dLidarPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  lidar_sensors_.clear();

  for (int i = 0; i < model->nsensor; ++i)
  {
    // Skip sensors that aren't mujoco plugins
    if (model->sensor_type[i] != mjtSensor::mjSENS_PLUGIN)
    {
      continue;
    }

    // Check if this plugin sensor is of the lidar plugin type
    int plugin_slot = model->plugin[model->sensor_plugin[i]];
    const mjpPlugin* plugin = mjp_getPluginAtSlot(plugin_slot);
    if (!plugin || std::string(plugin->name) != "mujoco.plugin.lidar")
    {
      continue;
    }

    // Add the sensor to our list, if possible
    if (!register_sensor(model, i))
    {
      return false;
    }
  }

  if (lidar_sensors_.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "No lidar sensors found");
  }

  return true;
}

bool Mujoco3dLidarPlugin::register_sensor(const mjModel* model, int sensor_idx)
{
  const auto sensor_name = mj_id2name(model, mjtObj::mjOBJ_SENSOR, sensor_idx);
  if (!sensor_name)
  {
    RCLCPP_WARN(node_->get_logger(), "Unnamed lidar sensor at index %d, skipping", sensor_idx);
    return true;
  }

  Lidar3dConfig lidar_config;
  lidar_config.name = sensor_name;
  lidar_config.sensor_id = sensor_idx;
  lidar_config.sensor_adr = model->sensor_adr[sensor_idx];

  auto plugin_inst = model->sensor_plugin[sensor_idx];
  lidar_config.plugin_stateadr = model->plugin_stateadr[plugin_inst];

  // Helper function to pull settings out of the plugin directly
  auto get_cfg = [&](const char* key) -> std::string {
    const char* val = mj_getPluginConfig(model, plugin_inst, key);
    return val ? std::string(val) : "";
  };

  // Resolution
  read_vector(lidar_config.resolution, get_cfg("resolution"));
  if (lidar_config.resolution.size() != 2 || lidar_config.resolution[0] <= 0 || lidar_config.resolution[1] <= 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid resolution for sensor '%s'", sensor_name);
    return false;
  }
  lidar_config.is_3d = lidar_config.resolution[1] > 1;

  // horizontal field of view
  read_vector(lidar_config.azimuth_range, get_cfg("azimuth_range"));

  // vertical field of view
  read_vector(lidar_config.elevation_range, get_cfg("elevation_range"));
  if (lidar_config.elevation_range.size() == 1)
  {
    lidar_config.elevation_range.push_back(lidar_config.elevation_range[0]);
  }
  else if (lidar_config.elevation_range.empty())
  {
    lidar_config.elevation_range = { 0.0, 0.0 };
  }

  // max/min ranges
  std::string max_range_str = get_cfg("max_range");
  lidar_config.range_max = max_range_str.empty() ? 1000.0 : std::atof(max_range_str.c_str());
  std::string min_range_str = get_cfg("min_range");
  lidar_config.range_min = min_range_str.empty() ? 0.0 : std::atof(min_range_str.c_str());
  if (lidar_config.range_max <= lidar_config.range_min)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid ranges for sensor '%s'", sensor_name);
    return false;
  }

  // Whether or not is is asynchronous
  std::string async_bool_str = get_cfg("async");
  if (!async_bool_str.empty())
  {
    // Anything non-zero is true
    lidar_config.async = (std::atoi(async_bool_str.c_str()) != 0);
  }

  // Grab the frame name and topic from the node parameters, the frame name will default to the
  // site name in the MJCF if not provided. Topics default to `/scan` and `/points` for 2-D and
  // 3-D scans, respectively.
  // Note that we must prefix our param declarations to ensure nesting works out properly.
  const std::string param_prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
  const auto site_name = mj_id2name(model, mjtObj::mjOBJ_SITE, model->sensor_objid[sensor_idx]);
  const std::string frame_name_param = param_prefix + lidar_config.name + ".frame_name";
  if (!node_->has_parameter(frame_name_param))
  {
    node_->declare_parameter(frame_name_param, site_name);
  }
  lidar_config.frame_name = node_->get_parameter(frame_name_param).as_string();

  const std::string lidar_topic_param = param_prefix + lidar_config.name + ".topic";
  if (!node_->has_parameter(lidar_topic_param))
  {
    node_->declare_parameter(lidar_topic_param, lidar_config.is_3d ? "/points" : "/scan");
  }
  lidar_config.lidar_topic = node_->get_parameter(lidar_topic_param).as_string();

  // Setup publishers and messages for the sensor type
  if (lidar_config.is_3d)
  {
    compute_vectors(lidar_config);

    // Point layout, following the ROS drivers of spinning lidars:
    //  - "xyz" (default): x, y, z (float32)
    //  - "robosense": x, y, z, intensity (float32), ring (uint16), timestamp (float64, absolute time in seconds),
    //    as rslidar_sdk and the Hesai driver
    //  - "velodyne": x, y, z, intensity (float32), ring (uint16), time (float32, seconds from the scan stamp),
    //    as velodyne_pointcloud
    //  - "ouster": x, y, z, intensity (float32), t (uint32, nanoseconds from the scan stamp), reflectivity (uint16),
    //    ring (uint16), ambient (uint16), range (uint32, millimetres), as the Ouster driver; rays without a return
    //    are (0, 0, 0) with range 0
    const std::string point_fields_param = param_prefix + lidar_config.name + ".point_fields";
    if (!node_->has_parameter(point_fields_param))
    {
      node_->declare_parameter(point_fields_param, "xyz");
    }
    const std::string point_fields = node_->get_parameter(point_fields_param).as_string();
    if (point_fields == "xyz")
    {
      lidar_config.point_layout = PointLayout::XYZ;
    }
    else if (point_fields == "robosense")
    {
      lidar_config.point_layout = PointLayout::ROBOSENSE;
    }
    else if (point_fields == "velodyne")
    {
      lidar_config.point_layout = PointLayout::VELODYNE;
    }
    else if (point_fields == "ouster")
    {
      lidar_config.point_layout = PointLayout::OUSTER;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid point_fields '%s' for sensor '%s', expected 'xyz', 'robosense', 'velodyne' or 'ouster'",
                   point_fields.c_str(), sensor_name);
      return false;
    }

    using sensor_msgs::msg::PointField;
    std::vector<std::pair<std::string, uint8_t>> fields = { { "x", PointField::FLOAT32 },
                                                            { "y", PointField::FLOAT32 },
                                                            { "z", PointField::FLOAT32 } };
    if (lidar_config.point_layout == PointLayout::ROBOSENSE)
    {
      fields.insert(fields.end(), { { "intensity", PointField::FLOAT32 },
                                    { "ring", PointField::UINT16 },
                                    { "timestamp", PointField::FLOAT64 } });
    }
    else if (lidar_config.point_layout == PointLayout::VELODYNE)
    {
      fields.insert(fields.end(), { { "intensity", PointField::FLOAT32 },
                                    { "ring", PointField::UINT16 },
                                    { "time", PointField::FLOAT32 } });
    }
    else if (lidar_config.point_layout == PointLayout::OUSTER)
    {
      fields.insert(fields.end(), { { "intensity", PointField::FLOAT32 },
                                    { "t", PointField::UINT32 },
                                    { "reflectivity", PointField::UINT16 },
                                    { "ring", PointField::UINT16 },
                                    { "ambient", PointField::UINT16 },
                                    { "range", PointField::UINT32 } });
    }
    const uint32_t point_step = set_aligned_fields(lidar_config.point_cloud_msg, fields);

    // Configure the static parameters of the pointcloud message
    lidar_config.point_cloud_msg.header.frame_id = lidar_config.frame_name;
    lidar_config.point_cloud_msg.width = lidar_config.resolution[0];
    lidar_config.point_cloud_msg.height = lidar_config.resolution[1];
    lidar_config.point_cloud_msg.point_step = point_step;
    lidar_config.point_cloud_msg.is_bigendian = is_big_endian();
    // Rays without a return are NaN, except in the Ouster layout where they are zeros
    lidar_config.point_cloud_msg.is_dense = lidar_config.point_layout == PointLayout::OUSTER;
    lidar_config.point_cloud_msg.row_step =
        lidar_config.point_cloud_msg.point_step * lidar_config.point_cloud_msg.width;
    lidar_config.point_cloud_msg.data.resize(lidar_config.point_cloud_msg.row_step *
                                             lidar_config.point_cloud_msg.height);

    lidar_config.pointcloud_pub_raw =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_config.lidar_topic, 1);
    lidar_config.pointcloud_pub = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::PointCloud2>>(
        lidar_config.pointcloud_pub_raw);
  }
  else
  {
    float angle_increment = static_cast<float>(lidar_config.azimuth_range[1] - lidar_config.azimuth_range[0]) /
                            static_cast<float>(lidar_config.resolution[0] - 1);

    // Configure the static parameters of the laserscan message
    lidar_config.laser_scan_msg.header.frame_id = lidar_config.frame_name;
    lidar_config.laser_scan_msg.time_increment = 0.0;
    lidar_config.laser_scan_msg.angle_min = static_cast<float>(lidar_config.azimuth_range[0]);
    lidar_config.laser_scan_msg.angle_max = static_cast<float>(lidar_config.azimuth_range[1]);
    lidar_config.laser_scan_msg.angle_increment = angle_increment;
    lidar_config.laser_scan_msg.range_min = static_cast<float>(lidar_config.range_min);
    lidar_config.laser_scan_msg.range_max = static_cast<float>(lidar_config.range_max);
    lidar_config.laser_scan_msg.ranges.resize(lidar_config.resolution[0] * lidar_config.resolution[1]);
    lidar_config.laser_scan_msg.intensities.resize(0);

    lidar_config.scan_pub_raw = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar_config.lidar_topic, 1);
    lidar_config.scan_pub =
        std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>>(lidar_config.scan_pub_raw);
  }

  // Note that we have added the sensor
  RCLCPP_INFO(node_->get_logger(), "Adding lidar sensor: '%s', idx: %d", lidar_config.name.c_str(),
              lidar_config.sensor_id);
  RCLCPP_INFO(node_->get_logger(), "         sensor_adr: '%d'", lidar_config.sensor_adr);
  RCLCPP_INFO(node_->get_logger(), "         frame_name: '%s'", lidar_config.frame_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "      azimuth_range: %.4f - %.4f", lidar_config.azimuth_range[0],
              lidar_config.azimuth_range[1]);
  RCLCPP_INFO(node_->get_logger(), "    elevation_range: %.4f - %.4f", lidar_config.elevation_range[0],
              lidar_config.elevation_range[1]);
  RCLCPP_INFO(node_->get_logger(), "         resolution: %d - %d", lidar_config.resolution[0],
              lidar_config.resolution[1]);
  RCLCPP_INFO(node_->get_logger(), "          range_min: %f", lidar_config.range_min);
  RCLCPP_INFO(node_->get_logger(), "          range_max: %f", lidar_config.range_max);
  RCLCPP_INFO(node_->get_logger(), "              topic: %s", lidar_config.lidar_topic.c_str());
  if (lidar_config.is_3d)
  {
    RCLCPP_INFO(node_->get_logger(), "       point_fields: %s", point_fields_name(lidar_config.point_layout));
  }
  RCLCPP_INFO(node_->get_logger(), "       asynchronous: %d", lidar_config.async);

  lidar_sensors_.push_back(std::move(lidar_config));
  return true;
}

void Mujoco3dLidarPlugin::update_write(const mjModel* /* model */, mjData* data)
{
  for (Lidar3dConfig& lidar : lidar_sensors_)
  {
    // Check for new sensor data, and skip if not yet computed, stale, or otherwise
    // invalid. This will explicitly skip the 0th timepoint, since that is not a valid
    // point for a scan, anyway.
    mjtNum compute_time = data->plugin_state[lidar.plugin_stateadr];
    if (!(compute_time > lidar.last_published_time))
    {
      continue;
    }
    lidar.last_published_time = compute_time;

    // If there is updated data, copy and publish the information
    int32_t sec = static_cast<int32_t>(std::floor(compute_time));
    uint32_t nsec = static_cast<uint32_t>((compute_time - sec) * 1e9);
    rclcpp::Time stamp(sec, nsec, RCL_ROS_TIME);

    const int num_rays = lidar.resolution[0] * lidar.resolution[1];
    const mjtNum* sensordata = data->sensordata + lidar.sensor_adr;

    // For 3d sensors, publish PointCloud messages
    if (lidar.is_3d && static_cast<int>(lidar.vectors.size()) == num_rays)
    {
      sensor_msgs::PointCloud2Iterator<float> iterX(lidar.point_cloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iterY(lidar.point_cloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iterZ(lidar.point_cloud_msg, "z");

      const float no_return =
          lidar.point_layout == PointLayout::OUSTER ? 0.0f : std::numeric_limits<float>::quiet_NaN();
      for (int i = 0; i < num_rays; ++i)
      {
        float dist = static_cast<float>(sensordata[i]);
        if (dist >= lidar.range_min && dist <= lidar.range_max)
        {
          *iterX = static_cast<float>(lidar.vectors[i].x * dist);
          *iterY = static_cast<float>(lidar.vectors[i].y * dist);
          *iterZ = static_cast<float>(lidar.vectors[i].z * dist);
        }
        else
        {
          *iterX = no_return;
          *iterY = no_return;
          *iterZ = no_return;
        }

        // Increment the iterators
        ++iterX;
        ++iterY;
        ++iterZ;
      }

      // Intensity, ring and per point time. The whole scan is cast at once, so every point is taken at the time
      // of the scan: the absolute timestamp is the stamp, the relative times are zero.
      const int width = lidar.resolution[0];
      auto has_return = [&](int i) { return sensordata[i] >= lidar.range_min && sensordata[i] <= lidar.range_max; };
      if (lidar.point_layout == PointLayout::ROBOSENSE || lidar.point_layout == PointLayout::VELODYNE)
      {
        sensor_msgs::PointCloud2Iterator<float> iterIntensity(lidar.point_cloud_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iterRing(lidar.point_cloud_msg, "ring");
        for (int i = 0; i < num_rays; ++i, ++iterIntensity, ++iterRing)
        {
          *iterIntensity = has_return(i) ? 1.0f : 0.0f;
          *iterRing = static_cast<uint16_t>(i / width);
        }
        if (lidar.point_layout == PointLayout::ROBOSENSE)
        {
          sensor_msgs::PointCloud2Iterator<double> iterTimestamp(lidar.point_cloud_msg, "timestamp");
          for (int i = 0; i < num_rays; ++i, ++iterTimestamp)
          {
            *iterTimestamp = static_cast<double>(compute_time);
          }
        }
        else
        {
          sensor_msgs::PointCloud2Iterator<float> iterTime(lidar.point_cloud_msg, "time");
          for (int i = 0; i < num_rays; ++i, ++iterTime)
          {
            *iterTime = 0.0f;
          }
        }
      }
      else if (lidar.point_layout == PointLayout::OUSTER)
      {
        sensor_msgs::PointCloud2Iterator<float> iterIntensity(lidar.point_cloud_msg, "intensity");
        sensor_msgs::PointCloud2Iterator<uint32_t> iterT(lidar.point_cloud_msg, "t");
        sensor_msgs::PointCloud2Iterator<uint16_t> iterReflectivity(lidar.point_cloud_msg, "reflectivity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iterRing(lidar.point_cloud_msg, "ring");
        sensor_msgs::PointCloud2Iterator<uint16_t> iterAmbient(lidar.point_cloud_msg, "ambient");
        sensor_msgs::PointCloud2Iterator<uint32_t> iterRange(lidar.point_cloud_msg, "range");
        for (int i = 0; i < num_rays;
             ++i, ++iterIntensity, ++iterT, ++iterReflectivity, ++iterRing, ++iterAmbient, ++iterRange)
        {
          const bool hit = has_return(i);
          *iterIntensity = hit ? 1.0f : 0.0f;
          *iterT = 0;
          *iterReflectivity = 0;
          *iterRing = static_cast<uint16_t>(i / width);
          *iterAmbient = 0;
          *iterRange = hit ? static_cast<uint32_t>(std::llround(sensordata[i] * 1000.0)) : 0;
        }
      }

      lidar.point_cloud_msg.header.stamp = stamp;
#if REALTIME_TOOLS_VERSION_MAJOR > 2
      lidar.pointcloud_pub->try_publish(lidar.point_cloud_msg);
#else
      lidar.pointcloud_pub->tryPublish(lidar.point_cloud_msg);
#endif
    }
    // For 2-D sensors, publish LaserScans
    else if (!lidar.is_3d)
    {
      for (int i = 0; i < num_rays; ++i)
      {
        lidar.laser_scan_msg.ranges[i] = static_cast<float>(sensordata[i]);
      }

      lidar.laser_scan_msg.header.stamp = stamp;
#if REALTIME_TOOLS_VERSION_MAJOR > 2
      lidar.scan_pub->try_publish(lidar.laser_scan_msg);
#else
      lidar.scan_pub->tryPublish(lidar.laser_scan_msg);
#endif
    }
  }
}

void Mujoco3dLidarPlugin::cleanup()
{
  lidar_sensors_.clear();
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::Mujoco3dLidarPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
