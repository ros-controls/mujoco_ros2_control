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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__FTS_GRAV_COMP_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__FTS_GRAV_COMP_PLUGIN_HPP_

#include <geometry_msgs/msg/wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <cstddef>
#include <mutex>
#include <queue>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Per-FTS bookkeeping -
 *
 * One instance exists per FTS registered by FtsGravCompPlugin::register_fts().
 */
struct FtsData
{
  // sensor name of FTS. Mujoco should have sensors <sensor_name>_force and <sensor_name>_torque
  std::string sensor_name;
  // mujoco sensor address for the force and torque sensors
  int sensor_adr_force{ -1 };
  int sensor_adr_torque{ -1 };
  // site id of the site used for center of gravity reference
  int cog_site_id{ -1 };
  // array of doubles of the center of gravity w.r.t. cog_site_it in meters for [x, y, z]
  mjtNum cog_pos[3];
  // mass that is being compensated in kg
  mjtNum cog_mass;
  // site id of the site used for the mujoco force and torque sensors
  int fts_site_id{ -1 };

  // Tare offset subtracted from the gravity-compensated force/torque every cycle. Set by
  // update_read() when a zero request for this FTS is drained from pending_zero_indices_.
  mjtNum force_offset[3]{ 0.0, 0.0, 0.0 };
  mjtNum torque_offset[3]{ 0.0, 0.0, 0.0 };
  // Set (under zero_mutex_) when update_read() drains a zero request for this FTS, and
  // consumed on the very next pass through the gravity-comp loop. Only ever touched from the
  // real-time update_read() thread, so it needs no synchronization of its own.
  bool zero_pending{ false };

  // Trigger service that tares this FTS, zeroing it based on the currently read values.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_service;
};

/**
 * @brief Plugin that runs gravity compensation on a force torque sensor
 */
class FtsGravCompPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  FtsGravCompPlugin() = default;
  ~FtsGravCompPlugin() override = default;

  /**
   * @brief Parses the params file and returns a vector of unique sensor names
   *
   */
  std::vector<std::string> get_sensor_names_from_parameters();

  /**
   * @brief Register force torque sensors from the model that match the params
   *
   */
  bool register_fts(const mjModel* model);

  /**
   * @brief Initialize the plugin
   */
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;

  /**
   * @brief Compensate the force/torque sensors in the state read by ros2_control
   */
  void update_read(const mjModel* model, mjData* data) override;

  /**
   * @brief Cleanup the plugin
   */
  void cleanup() override;

  /**
   * @brief getter for fts data used for testing
   */
  std::vector<FtsData> get_fts_data() const
  { return fts_; }

private:
  /**
   * @brief Service callback that requests a tare of the given FTS. The actual tare is performed
   * on the real-time thread inside update_read()
   */
  void register_zero_fts(std::size_t fts_index, const std_srvs::srv::Trigger::Request::SharedPtr request,
                         std_srvs::srv::Trigger::Response::SharedPtr response);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("FtsGravCompPlugin");

  std::vector<FtsData> fts_;

  // thread safety tools for zeroing ft sensor
  std::atomic<bool> zero_pending_{ false };
  std::mutex zero_mutex_;
  std::queue<std::size_t> pending_zero_indices_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__FTS_GRAV_COMP_PLUGIN_HPP_
