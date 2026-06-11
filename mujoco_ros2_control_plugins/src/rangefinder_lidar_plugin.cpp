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

#include "mujoco_ros2_control_plugins/rangefinder_lidar_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

std::pair<std::string, int> RangefinderLidarPlugin::parse_lidar_name(const std::string& sensor_name)
{
  const auto split_idx = sensor_name.find_last_of("-");
  if (split_idx == std::string::npos)
  {
    return { sensor_name, -1 };
  }

  const auto lidar_name = sensor_name.substr(0, split_idx);
  const auto lidar_idx = sensor_name.substr(split_idx + 1);
  if (lidar_idx.empty() || !std::all_of(lidar_idx.begin(), lidar_idx.end(), ::isdigit))
  {
    return { lidar_name, -1 };
  }

  return { lidar_name, std::stoi(lidar_idx) };
}

bool RangefinderLidarPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  model_ = model;
  lidar_sensors_.clear();

  const std::string param_prefix = "mujoco_plugins.rangefinder_lidar_plugin.";

  // Read publish rate
  if (!node_->has_parameter(param_prefix + "lidar_publish_rate"))
  {
    node_->declare_parameter(param_prefix + "lidar_publish_rate", publish_rate_);
  }
  publish_rate_ = node_->get_parameter(param_prefix + "lidar_publish_rate").as_double();

  // Iterate over sensors and identify rangefinders, grouping by name
  for (int i = 0; i < model->nsensor; ++i)
  {
    if (model->sensor_type[i] != mjtSensor::mjSENS_RANGEFINDER)
    {
      continue;
    }

    const auto sensor_name_maybe = mj_id2name(model, mjtObj::mjOBJ_SENSOR, i);
    if (!sensor_name_maybe)
    {
      RCLCPP_WARN(node_->get_logger(), "Cannot find a name for rangefinder sensor at index %d, skipping", i);
      continue;
    }
    const std::string sensor_name(sensor_name_maybe);

    const auto [lidar_name, idx] = parse_lidar_name(sensor_name);
    if (idx == -1)
    {
      RCLCPP_WARN(node_->get_logger(), "Failed to parse rangefinder sensor name: '%s', skipping", sensor_name.c_str());
      continue;
    }

    // Find or create this lidar group
    auto lidar_it = std::find_if(lidar_sensors_.begin(), lidar_sensors_.end(),
                                 [&lidar_name](const RangefinderLidarData& data) { return data.name == lidar_name; });

    if (lidar_it == lidar_sensors_.end())
    {
      if (!register_sensor(lidar_name, model))
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to register rangefinder lidar: '%s'", lidar_name.c_str());
        return false;
      }
      lidar_it = lidar_sensors_.end() - 1;
    }

    // Map this rangefinder's sensordata address
    if (idx >= 0 && idx < static_cast<int>(lidar_it->sensor_indexes.size()))
    {
      lidar_it->sensor_indexes[idx] = model->sensor_adr[i];
    }
  }

  if (lidar_sensors_.empty())
  {
    RCLCPP_INFO(node_->get_logger(), "No rangefinder lidar sensors found");
    return true;
  }

  // Allocate raw data buffers and start the publish thread
  for (auto& lidar : lidar_sensors_)
  {
    lidar.raw_data.resize(lidar.num_rangefinders, 0.0);
  }

  RCLCPP_INFO(node_->get_logger(), "Publishing lidar at %f hertz", publish_rate_);

  return true;
}

bool RangefinderLidarPlugin::register_sensor(const std::string& lidar_name, const mjModel* model)
{
  const std::string ns = "mujoco_plugins.rangefinder_lidar_plugin." + lidar_name + ".";

  // Check required parameters
  if (!node_->has_parameter(ns + "frame_name"))
  {
    RCLCPP_ERROR(node_->get_logger(), "Required parameter '%sframe_name' not found for lidar '%s'", ns.c_str(),
                 lidar_name.c_str());
    return false;
  }
  if (!node_->has_parameter(ns + "min_angle") || !node_->has_parameter(ns + "max_angle") ||
      !node_->has_parameter(ns + "angle_increment"))
  {
    RCLCPP_ERROR(node_->get_logger(), "Required angle parameters not found for lidar '%s'", lidar_name.c_str());
    return false;
  }

  // Declare optional parameters with defaults
  if (!node_->has_parameter(ns + "range_min"))
    node_->declare_parameter(ns + "range_min", 0.0);
  if (!node_->has_parameter(ns + "range_max"))
    node_->declare_parameter(ns + "range_max", 1000.0);
  if (!node_->has_parameter(ns + "laserscan_topic"))
    node_->declare_parameter(ns + "laserscan_topic", std::string("/scan"));

  // Setup lidar data
  RangefinderLidarData lidar;
  lidar.name = lidar_name;
  lidar.frame_name = node_->get_parameter(ns + "frame_name").as_string();
  lidar.min_angle = node_->get_parameter(ns + "min_angle").as_double();
  lidar.max_angle = node_->get_parameter(ns + "max_angle").as_double();
  lidar.angle_increment = node_->get_parameter(ns + "angle_increment").as_double();
  lidar.num_rangefinders = static_cast<int>((lidar.max_angle - lidar.min_angle) / lidar.angle_increment) + 1;
  lidar.laserscan_topic = node_->get_parameter(ns + "laserscan_topic").as_string();
  lidar.range_min = node_->get_parameter(ns + "range_min").as_double();
  lidar.range_max = node_->get_parameter(ns + "range_max").as_double();
  lidar.sensor_indexes.resize(lidar.num_rangefinders, 0);

  // Configure static message fields
  lidar.laser_scan_msg.header.frame_id = lidar.frame_name;
  lidar.laser_scan_msg.time_increment = 0.0;
  lidar.laser_scan_msg.scan_time = 1.0f / static_cast<float>(publish_rate_);
  lidar.laser_scan_msg.angle_min = static_cast<float>(lidar.min_angle);
  lidar.laser_scan_msg.angle_max = static_cast<float>(lidar.max_angle);
  lidar.laser_scan_msg.angle_increment = static_cast<float>(lidar.angle_increment);
  lidar.laser_scan_msg.range_min = static_cast<float>(lidar.range_min);
  lidar.laser_scan_msg.range_max = static_cast<float>(lidar.range_max);
  lidar.laser_scan_msg.ranges.resize(lidar.num_rangefinders);
  lidar.laser_scan_msg.intensities.resize(0);

  // Initialize publisher
  lidar.scan_pub_raw = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar.laserscan_topic, 1);
  lidar.scan_pub = std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>>(lidar.scan_pub_raw);

  RCLCPP_INFO(node_->get_logger(), "Adding rangefinder lidar: '%s'", lidar.name.c_str());
  RCLCPP_INFO(node_->get_logger(), "    frame_name: '%s'", lidar.frame_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "    num_rangefinders: %d", lidar.num_rangefinders);
  RCLCPP_INFO(node_->get_logger(), "    min_angle: %f", lidar.min_angle);
  RCLCPP_INFO(node_->get_logger(), "    max_angle: %f", lidar.max_angle);
  RCLCPP_INFO(node_->get_logger(), "    angle_increment: %f", lidar.angle_increment);
  RCLCPP_INFO(node_->get_logger(), "    range_min: %f", lidar.range_min);
  RCLCPP_INFO(node_->get_logger(), "    range_max: %f", lidar.range_max);
  RCLCPP_INFO(node_->get_logger(), "    topic: %s", lidar.laserscan_topic.c_str());

  lidar_sensors_.push_back(std::move(lidar));
  return true;
}

void RangefinderLidarPlugin::update(const mjModel* /*model*/, mjData* data)
{
  if (lidar_sensors_.empty())
  {
    return;
  }

  auto now = node_->get_clock()->now();
  if ((now - last_publish_time_).seconds() < (1.0 / publish_rate_))
  {
    return;
  }
  last_publish_time_ = now;

  for (auto& lidar : lidar_sensors_)
  {
    for (size_t idx = 0; idx < lidar.sensor_indexes.size(); ++idx)
    {
      float range = static_cast<float>(data->sensordata[lidar.sensor_indexes[idx]]);
      lidar.laser_scan_msg.ranges[idx] = (range < lidar.range_min || range > lidar.range_max) ? -1.0f : range;
    }
    lidar.laser_scan_msg.header.stamp = now;

#if REALTIME_TOOLS_VERSION_MAJOR > 2
    lidar.scan_pub->try_publish(lidar.laser_scan_msg);
#else
    lidar.scan_pub->tryPublish(lidar.laser_scan_msg);
#endif
  }
}

void RangefinderLidarPlugin::cleanup()
{
  lidar_sensors_.clear();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::RangefinderLidarPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
