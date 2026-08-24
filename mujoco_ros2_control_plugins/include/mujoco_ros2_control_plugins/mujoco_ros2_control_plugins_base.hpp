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
 * Plugins extend mujoco_ros2_control with custom behavior, e.g. publishing extra topics or
 * applying external forces.
 *
 * Two optional hooks are available. Override whichever fits your use case:
 *
 * - `update()` runs on the ros2_control control thread, once per `write()` cycle. `data` is a
 *   recent snapshot, not the live `mj_data_`. Use this for anything that doesn't need to run on
 *   exactly one physics step, e.g. publishing sensor data or servicing a trigger.
 * - `pre_step()` runs on the physics thread, immediately before every `mj_step()`, with direct
 *   access to the live `mj_data_`. Use this for anything that must hold for exactly one step,
 *   e.g. a hard kinematic override. There's no command buffer: an untouched entry keeps its
 *   last value, so releasing a command (e.g. an expired force) needs an explicit write. This
 *   should be used with caution!
 *
 * @note Both hooks must avoid blocking or the control loop or simulation itself will be held up!
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
   * @brief Called once per control cycle, on the control thread. Default does nothing.
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to a recent snapshot (not the live `mj_data_`)
   */
  virtual void update(const mjModel* /*model*/, mjData* /*data*/)
  {
  }

  /**
   * @brief Called immediately before every simulation step on the physics thread. Default does nothing.
   *
   * USE WITH CAUTION.
   *
   * @param data Pointer to the live MuJoCo data, can be modified as needed.
   */
  virtual void pre_step(mjData* /*data*/)
  {
  }

  /**
   * @brief Cleanup the plugin
   */
  virtual void cleanup() = 0;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
