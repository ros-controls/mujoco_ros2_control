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
 * @brief Isolated data container for passing additional information from a plugin to the simulation.
 *
 * This structure is made available to plugins during `update`, and can be used to pass data from
 * plugins through to the underlying simulation in an extensible, well defined manner.
 *
 * For now, we include `xfrc_applied` as a mechanism to inject external forces directly into the physics
 * simulation, while providing rendering capabilities that work in tandem with the Simulate app's
 * force rendering arrows. Of note, these buffers exist separately from `mj_data_` and `mj_data_control_`
 * as the Simulate's render thread's `Sync()` function can zero out applied Cartesian forces, which
 * otherwise complicates determining desired forces from external plugins.
 *
 * @param xfrc_applied Applied Cartesian force/torque (nbody x 6)
 */
struct PluginData
{
  std::vector<mjtNum> xfrc_applied;

  /**
   * @brief Reserves sufficient data for vectors based on the provided model.
   */
  void allocate(const mjModel* m)
  {
    xfrc_applied.assign(6 * m->nbody, 0.0);
  }

  /**
   * @brief Sets all entries to 0 in the provided buffer.
   */
  void clear()
  {
    std::fill(xfrc_applied.begin(), xfrc_applied.end(), 0.0);
  }
};

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
   * @param data Pointer to the MuJoCo control data. Plugins may read state any value as required,
   *        and write to `data->ctrl` and `data->qfrc_applied`. These fields have already
   *        been populated by the HW interface, so plugins can override or add to them immediately
   *        before they are sent to the physics simulation.
   * @param plugin_data Reference to the simulation's PluginData container, which currently
   *        allows users to input Cartesian forces to the underlying simulation through
   *        `plugin_data->xfrc_applied`.
   * @note Runs after the HW interface has written its commands, so plugin writes to ctrl
   *       and qfrc_applied take priority. Changes are copied to the physics data before
   *       the next mj_step.
   * @note This method will be called at the end of the mujoco_ros2_control read loop, before the update loop of
   * controllers and the write loop. This means that changes to the data here will be visible to controllers and will
   * affect the next simulation step.
   * @note This method will be called in a real-time thread, so it should avoid blocking operations and should be
   * efficient.
   */
  virtual void update(const mjModel* model, mjData* data, std::shared_ptr<PluginData> plugin_data) = 0;

  /**
   * @brief Cleanup the plugin
   */
  virtual void cleanup() = 0;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
