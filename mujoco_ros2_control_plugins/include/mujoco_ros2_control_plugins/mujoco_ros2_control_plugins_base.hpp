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
 * Four optional hooks are available. Override whichever fits your use case:
 *
 * - `update_read()` runs on the ros2_control control thread, once per `read()` cycle, before the
 *   state is handed to ros2_control. `data` holds only the per-step control state (`time`, `qpos`,
 *   `qvel`, `act`, `qfrc_actuator`, `sensordata`, `ctrl`); everything else is stale. Plugins run in
 *   order and each sees the previous one's changes. Use this to post-process states or sensors,
 *   e.g. compensating a force/torque sensor.
 * - `update_write()` runs on the ros2_control control thread, once per `write()` cycle. `data` is a
 *   recent snapshot, not the live `mj_data_`. Use this for anything that doesn't need to run on
 *   exactly one physics step, e.g. publishing sensor data or servicing a trigger.
 * - `pre_step()` runs on the physics thread, immediately before every `mj_step()`, with direct
 *   access to the live `mj_data_`. Use this for anything that must hold for exactly one step,
 *   e.g. a hard kinematic override. There's no command buffer: an untouched entry keeps its
 *   last value, so releasing a command (e.g. an expired force) needs an explicit write. This
 *   should be used with caution!
 * - `on_reset()` runs immediately after a world reset, with the live `mj_data_` in its
 *   post-reset state. Use this to clear plugin state that refers to the pre-reset world,
 *   e.g. a latched command.
 *
 * @note All hooks must avoid blocking or the control loop or simulation itself will be held up!
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
   * @brief Called once per `read()` cycle, on the control thread, before the state is handed to
   * ros2_control. Default does nothing.
   *
   * Only the control state fields of `data` are valid (`time`, `qpos`, `qvel`, `act`,
   * `qfrc_actuator`, `sensordata`, `ctrl`), all from the same physics step. Derived quantities
   * (e.g. `site_xpos`) are stale: call `mj_kinematics()` or similar on `data` if needed.
   * Changes to the control state fields are what ros2_control reads.
   *
   * @param model Pointer to the MuJoCo model
   * @param data Scratch data holding the latest control state, modify it in place
   */
  virtual void update_read(const mjModel* /*model*/, mjData* /*data*/)
  {
  }

  /**
   * @brief Called once per `write()` cycle, on the control thread, after the hardware interface
   * has staged the controller commands and right before they are sent to the simulation.
   *
   * Plugins run in order and can read and rewrite `ctrl`/`qfrc_applied`. The default forwards to
   * the deprecated `update()` so plugins that still override it keep working.
   *
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to a recent snapshot (not the live `mj_data_`)
   */
  virtual void update_write(const mjModel* model, mjData* data)
  {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    update(model, data);
#pragma GCC diagnostic pop
  }

  /**
   * @brief Same as `update_write()`. Default does nothing.
   * @deprecated Override `update_write()` instead, to keep it distinct from `update_read()`.
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to a recent snapshot (not the live `mj_data_`)
   */
  [[deprecated("Override update_write() instead.")]] virtual void update(const mjModel* /*model*/, mjData* /*data*/)
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
   * @brief Called after a world reset (ResetWorld service or UI reset). Default does nothing.
   *
   * @param data Pointer to the live MuJoCo data in its post-reset state.
   */
  virtual void on_reset(mjData* /*data*/)
  {
  }

  /**
   * @brief Cleanup the plugin
   */
  virtual void cleanup() = 0;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
