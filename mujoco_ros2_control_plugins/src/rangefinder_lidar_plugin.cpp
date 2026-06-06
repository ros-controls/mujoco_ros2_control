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

  const std::string param_prefix = "mujoco_plugins.mujoco_rangefinder_plugin.";

  // Read publish rate
  if (!node_->has_parameter(param_prefix + "publish_rate"))
  {
    node_->declare_parameter(param_prefix + "publish_rate", publish_rate_);
  }
  publish_rate_ = node_->get_parameter(param_prefix + "publish_rate").as_double();

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

  running_ = true;
  publish_thread_ = std::thread(&RangefinderLidarPlugin::publish_loop, this);

  return true;
}

bool RangefinderLidarPlugin::register_sensor(const std::string& lidar_name, const mjModel* model)
{
  const std::string param_prefix = "mujoco_plugins.mujoco_rangefinder_plugin.";
  const std::string param_ns = param_prefix + lidar_name + ".";

  RangefinderLidarData lidar;
  lidar.name = lidar_name;

  // Required parameters
  auto declare_and_get = [&](const std::string& key, const std::string& default_val = "") -> std::string {
    const std::string full = param_ns + key;
    if (!node_->has_parameter(full))
    {
      if (default_val.empty())
      {
        RCLCPP_ERROR(node_->get_logger(), "Required parameter '%s' not found for lidar '%s'", key.c_str(),
                     lidar_name.c_str());
        return "";
      }
      node_->declare_parameter(full, default_val);
    }
    return node_->get_parameter(full).as_string();
  };

  auto frame_name = declare_and_get("frame_name");
  auto min_angle = declare_and_get("min_angle");
  auto max_angle = declare_and_get("max_angle");
  auto angle_increment = declare_and_get("angle_increment");

  if (frame_name.empty() || min_angle.empty() || max_angle.empty() || angle_increment.empty())
  {
    return false;
  }

  lidar.frame_name = frame_name;
  lidar.min_angle = std::stod(min_angle);
  lidar.max_angle = std::stod(max_angle);
  lidar.angle_increment = std::stod(angle_increment);
  lidar.num_rangefinders = static_cast<int>((lidar.max_angle - lidar.min_angle) / lidar.angle_increment) + 1;

  auto topic = declare_and_get("topic", "/scan");
  auto range_min = declare_and_get("range_min", "0.0");
  auto range_max = declare_and_get("range_max", "1000.0");

  lidar.laserscan_topic = topic;
  lidar.range_min = std::stod(range_min);
  lidar.range_max = std::stod(range_max);
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

  lidar.scan_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar.laserscan_topic, 1);

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
  bool any_new = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (auto& lidar : lidar_sensors_)
    {
      for (size_t idx = 0; idx < lidar.sensor_indexes.size(); ++idx)
      {
        lidar.raw_data[idx] = data->sensordata[lidar.sensor_indexes[idx]];
      }
      any_new = true;
    }
    if (any_new)
    {
      new_data_ = true;
    }
  }
  if (any_new)
  {
    data_cv_.notify_one();
  }
}

void RangefinderLidarPlugin::publish_loop()
{
  while (running_)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);
    data_cv_.wait(lock, [this] { return new_data_ || !running_; });

    if (!running_)
    {
      break;
    }
    new_data_ = false;

    for (auto& lidar : lidar_sensors_)
    {
      for (size_t idx = 0; idx < lidar.raw_data.size(); ++idx)
      {
        float range = static_cast<float>(lidar.raw_data[idx]);
        lidar.laser_scan_msg.ranges[idx] = (range < lidar.range_min || range > lidar.range_max) ? -1.0f : range;
      }

      lidar.laser_scan_msg.header.stamp = node_->now();
      lidar.scan_pub->publish(lidar.laser_scan_msg);
    }
  }
}

void RangefinderLidarPlugin::cleanup()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    running_ = false;
  }
  data_cv_.notify_one();
  if (publish_thread_.joinable())
  {
    publish_thread_.join();
  }
  lidar_sensors_.clear();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::RangefinderLidarPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
