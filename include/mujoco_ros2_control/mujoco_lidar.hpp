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
#include <mutex>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace mujoco_ros2_control
{

struct LidarData
{
  std::string name;
  std::string frame_name;
  int num_rangefinders;
  double min_angle;
  double max_angle;
  double angle_increment;
  double range_min;
  double range_max;

  // Maps the index of the rangefinder to the index of the mujoco rangefinder's data.
  // E.g. lidar-034 -> sensor_indexes[34] will contain index of that rangefinder in mj_data_->sensordata
  std::vector<int> sensor_indexes;

  // For message publishing
  std::string laserscan_topic;
  sensor_msgs::msg::LaserScan laser_scan_msg;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
};

/**
 * @brief Wraps Mujoco rangefinder sensors for publishing ROS 2 LaserScan messages.
 */
class MujocoLidar
{
public:
  /**
   * @brief Constructs a new MujocoLidar wrapper object.
   *
   * @param node Will be used to construct laserscan publishers
   * @param sim_mutex Provides synchronized access to the mujoco_data object for grabbing rangefinder data
   * @param mujoco_data Mujoco data for the simulation
   * @param mujoco_model Mujoco model for the simulation
   * @param lidar_publish_rate The rate to publish all camera images, for now all images are published at the same rate.
   */
  explicit MujocoLidar(rclcpp::Node::SharedPtr& node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                       mjModel* mujoco_model, double lidar_publish_rate);

  /**
   * @brief Starts the lidar processing thread in the background.
   */
  void init();

  /**
   * @brief Stops the lidar processing thread and closes the relevant objects, call before shutdown.
   */
  void close();

  /**
   * @brief Parses lidar information from the mujoco model.
   *
   * Returns true if successful, false otherwise.
   */
  bool register_lidar(const hardware_interface::HardwareInfo& hardware_info);

private:
  /**
   * @brief Start publishing loop.
   */
  void update_loop();

  /**
   * @brief Updates the camera images and publishes info, images, and depth maps.
   */
  void update();

  rclcpp::Node::SharedPtr node_;

  // Ensures locked access to simulation data for rendering.
  std::recursive_mutex* sim_mutex_{ nullptr };

  mjData* mj_data_;
  mjModel* mj_model_;

  // Vector container to copy sensordata out of mj_data_
  std::vector<mjtNum> mj_lidar_data_;

  // LaserScan publishing rate in Hz
  double lidar_publish_rate_;

  // Rendering options for the cameras, currently hard coded to defaults
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  // Containers for ladar data and ROS constructs
  std::vector<LidarData> lidar_sensors_;

  // Lidar processing thread
  std::thread rendering_thread_;
  std::atomic_bool publish_lidar_;
};

}  // namespace mujoco_ros2_control
