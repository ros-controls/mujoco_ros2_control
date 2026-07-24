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

/**
 * @brief Per-lidar bookkeeping for a single named group of MuJoCo rangefinder sensors.
 *
 * One instance is created per distinct lidar name (see RangefinderLidarPlugin::parse_lidar_name),
 * grouping together every mjSENS_RANGEFINDER sensor that shares that name prefix into a single
 * LaserScan publisher.
 */
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
  /**
   * @brief Reads parameters for one lidar group and registers its LaserScan publisher.
   * @param lidar_name Name of the lidar group (the parameter namespace under
   * `mujoco_plugins.rangefinder_lidar_plugin.<lidar_name>`).
   * @param model Pointer to the MuJoCo model.
   * @return true if the required parameters were found and the lidar was registered, false otherwise.
   */
  bool register_sensor(const std::string& lidar_name, const mjModel* model);

  /**
   * @brief Publishes the buffered LaserScan message for every registered lidar group.
   */
  void publish_loop();

  /**
   * @brief Splits a MuJoCo rangefinder sensor name into its lidar group name and ray index.
   *
   * Expects the legacy naming convention `<lidar_name>-<index>` (e.g. `lidar-034` maps to
   * group name `lidar` and index `34`). If the name has no `-`, the whole string is the group
   * name and the index is -1. If it has a `-` but the suffix is not a plain integer, the index
   * is still -1 but the group name is only the part before the last `-`.
   *
   * @param sensor_name Name of the MuJoCo sensor, as declared in the MJCF.
   * @return A pair of (lidar group name, ray index), with index -1 on a parse failure.
   */
  static std::pair<std::string, int> parse_lidar_name(const std::string& sensor_name);

  rclcpp::Node::SharedPtr node_;
  const mjModel* model_{ nullptr };
  double publish_rate_{ 5.0 };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };

  std::vector<RangefinderLidarData> lidar_sensors_;
};

}  // namespace mujoco_ros2_control_plugins
