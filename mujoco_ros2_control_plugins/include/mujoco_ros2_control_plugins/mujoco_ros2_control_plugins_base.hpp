// Copyright 2026 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Base class for MuJoCo ROS 2 control plugins
 *
 * This is an example base class that plugins can inherit from.
 * Plugins can extend the functionality of mujoco_ros2_control
 * by implementing custom behaviors.
 */
class MuJoCoROS2ControlPluginBase
{
public:
  virtual ~MuJoCoROS2ControlPluginBase() = default;

  /**
   * @brief Initialize the plugin
   * @param node Shared pointer to the ROS 2 node for accessing parameters
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to the MuJoCo data
   * @return true if initialization was successful
   * @note This method will be called once when the plugin is loaded. It can be used to read parameters, set up
   * publishers/subscribers, etc. The node will be a child of the main mujoco_ros2_control node, so parameters should be
   * namespaced accordingly.
   */
  virtual bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) = 0;

  /**
   * @brief Update the plugin (called every simulation step)
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to the MuJoCo data
   * @note This method will be called at the end of the mujoco_ros2_control read loop, before the update loop of
   * controllers and the write loop. This means that changes to the data here will be visible to controllers and will
   * affect the next simulation step.
   * @note This method will be called in a real-time thread, so it should avoid blocking operations and should be
   * efficient.
   */
  virtual void update(const mjModel* model, mjData* data) = 0;

  /**
   * @brief Cleanup the plugin
   */
  virtual void cleanup() = 0;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
