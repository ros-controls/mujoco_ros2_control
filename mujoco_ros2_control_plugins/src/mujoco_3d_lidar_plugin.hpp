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

#pragma once

#include <vector>

#include <mujoco/mujoco.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Data container for lidar sensor information.
 */
struct Lidar3dConfig
{
  std::string name;
  int sensor_id;
  int sensor_adr;
  int plugin_stateadr;
  std::string frame_name;
  std::vector<int> resolution;
  std::vector<double> azimuth_range;
  std::vector<double> elevation_range;
  std::vector<geometry_msgs::msg::Vector3> vectors;
  double range_min;
  double range_max;
  bool is_3d;

  // Whether or not the underlying extension is async
  bool async{ false };

  // For message publishing
  std::string lidar_topic;

  // Tracking the last update and compute time from the plugin
  mjtNum last_compute_time{ -1.0 };
  mjtNum last_published_time{ -1.0 };

  sensor_msgs::msg::LaserScan laser_scan_msg;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_raw;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>> scan_pub;

  sensor_msgs::msg::PointCloud2 point_cloud_msg;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_raw;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::PointCloud2>> pointcloud_pub;
};

/**
 * @brief Plugin wrapper of the included 3D Lidar MuJoCo extension.
 *
 * Will convert 3d_lidar sensor data from the physics simulation into the appropriate
 * message and publish to the configured topic. Supports both 2-D and 3-D lidar
 * sensor configs, and will publish either LaserScan or PointCloud messages according
 * to the sensor type.
 */
class Mujoco3dLidarPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  /**
   * @brief Initializes a new MujocoLidar wrapper object for 3d lidar sensors.
   *
   * @param node Will be used to construct laserscan publishers
   * @param model MuJoCo model for the simulation
   * @param data MuJoCo data for the simulation
   */
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;

  /**
   * @brief Callback to update the plugin's data.
   *
   * Iterates over each lidar sensor and checks if new data is available. If not, this does
   * nothing. If so, this will copy the sensor data into a relevant message type and
   * publish it.
   */
  void update(const mjModel* model, mjData* data) override;

  /**
   * @brief Destructs the plugin and all relevant data.
   */
  void cleanup() override;

private:
  /**
   * @brief Helper function to construct data given the provided index and mjModel.
   *
   * @param model MuJoCo model for the simulation
   * @param sensor_idx Index of the sensor in the MuJoCo model
   * @return true
   * @return false
   */
  bool register_sensor(const mjModel* model, int sensor_idx);

  rclcpp::Node::SharedPtr node_;

  // Containers for lidar data and ROS constructs
  std::vector<Lidar3dConfig> lidar_sensors_;
};

}  // namespace mujoco_ros2_control_plugins
