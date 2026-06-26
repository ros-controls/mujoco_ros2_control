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

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>

namespace mujoco_ros2_control_plugins
{

struct RangefinderLidarData
{
  std::string name;
  std::string frame_name;
  int num_rangefinders;
  double min_angle;
  double max_angle;
  double angle_increment;
  double range_min;
  double range_max;

  // Maps the index of the rangefinder to the index of the MuJoCo rangefinder's data.
  // E.g. lidar-034 -> sensor_indexes[34] will contain index of that rangefinder in mj_data_->sensordata
  std::vector<int> sensor_indexes;

  // Raw data copied from mjData each update
  std::vector<mjtNum> raw_data;

  // For message publishing
  std::string laserscan_topic;
  sensor_msgs::msg::LaserScan laser_scan_msg;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_raw;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>> scan_pub;
};

/**
 * @brief Plugin wrapper for MuJoCo rangefinder-based lidar sensors.
 *
 * This handles the legacy pattern where individual mjSENS_RANGEFINDER sensors
 * are grouped by name convention (e.g. lidar_name-0, lidar_name-1, ...) and
 * published as LaserScan messages.
 */
class [[deprecated("We recommend using the 3d Lidar plugin instead."
                   "See documentation for migration instructions.")]] RangefinderLidarPlugin
  : public MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  bool register_sensor(const std::string& lidar_name, const mjModel* model);
  void publish_loop();

  static std::pair<std::string, int> parse_lidar_name(const std::string& sensor_name);

  rclcpp::Node::SharedPtr node_;
  const mjModel* model_{ nullptr };
  double publish_rate_{ 5.0 };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };

  std::vector<RangefinderLidarData> lidar_sensors_;
};

}  // namespace mujoco_ros2_control_plugins
