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
  int cog_site_id;
  // array of doubles of the center of gravity w.r.t. cog_site_it in meters for [x, y, z]
  mjtNum cog_pos[3];
  // mass that is being compensated in kg
  mjtNum cog_mass;
  // site id of the site used for the mujoco force and torque sensors
  int fts_site_id{ -1 };
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
   * @brief Update the plugin (called every simulation step)
   */
  void update(const mjModel* model, mjData* data) override;

  /**
   * @brief Cleanup the plugin
   */
  void cleanup() override;

  /**
   * @brief getter for fts data used for testing
   */
  std::vector<FtsData> get_fts_data() const
  {
    return fts_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("FtsGravCompPlugin");

  std::vector<FtsData> fts_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__FTS_GRAV_COMP_PLUGIN_HPP_
