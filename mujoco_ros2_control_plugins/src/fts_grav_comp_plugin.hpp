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
 * @brief Simple plugin that publishes a heartbeat message every second
 */
class FtsGravCompPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  FtsGravCompPlugin() = default;
  ~FtsGravCompPlugin() override = default;

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

private:
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr fts_wrench_publisher_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("FtsGravCompPlugin");
  rclcpp::Time last_publish_time_;
  rclcpp::Duration publish_period_{ 1, 0 };  // Publish every 1 second
  uint64_t message_count_{ 0 };
  int sensor_adr_force_{ -1 };
  int sensor_adr_torque_{ -1 };
  int sensor_dim_force_{ -1 };
  int sensor_dim_torque_{ -1 };
  geometry_msgs::msg::Wrench wrench_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__FTS_GRAV_COMP_PLUGIN_HPP_
