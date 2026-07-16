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

#include <cmath>
#include <limits>
#include <sstream>

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

  // Grab the frame name and topic from the node parameters, the frame name will default to the
  // site name in the MJCF if not provided. Topics default to `/scan` and `/points` for 2-D and
  // 3-D scans, respectively.
  // Note that we must prefix our param declarations to ensure nesting works out properly.
  const std::string param_prefix = "mujoco_plugins.mujoco_3d_lidar_plugin.";
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
    sensor_msgs::PointCloud2Modifier modifier(lidar_config.point_cloud_msg);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                  sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);

    // Configure the static parameters of the pointcloud message
    lidar_config.point_cloud_msg.header.frame_id = lidar_config.frame_name;
    lidar_config.point_cloud_msg.width = lidar_config.resolution[0];
    lidar_config.point_cloud_msg.height = lidar_config.resolution[1];
    lidar_config.point_cloud_msg.point_step = 12;
    lidar_config.point_cloud_msg.is_bigendian = is_big_endian();
    lidar_config.point_cloud_msg.is_dense = false;
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

  lidar_sensors_.push_back(std::move(lidar_config));
  return true;
}

void Mujoco3dLidarPlugin::update(const mjModel* /* model */, mjData* data)
{
  for (Lidar3dConfig& lidar : lidar_sensors_)
  {
    // Check for new sensor data, and skip if not yet computed or if it is stale
    mjtNum compute_time = data->plugin_state[lidar.plugin_stateadr];
    if (compute_time < 0.0 || compute_time == lidar.last_published_time)
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
          *iterX = std::numeric_limits<float>::quiet_NaN();
          *iterY = std::numeric_limits<float>::quiet_NaN();
          *iterZ = std::numeric_limits<float>::quiet_NaN();
        }

        // Increment the iterators
        ++iterX;
        ++iterY;
        ++iterZ;
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
