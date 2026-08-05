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

#include <typeinfo>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Base class for MuJoCo ROS 2 control plugins
 *
 * Plugins extend the functionality of mujoco_ros2_control by implementing custom behaviours,
 * such as publishing extra sensor topics or applying external forces to the simulation.
 *
 * Reads and writes are deliberately separated:
 *
 * - **Reading** simulation state is done through `get_sim_data()` and `get_mujoco_model()`, which
 *   expose the live physics containers as const pointers.
 * - **Writing** commands is done through the `control_data` buffer handed to `update()`. That
 *   buffer is *write-only*: every commandable field arrives filled with NaN, and only the entries
 *   a plugin actually writes are merged back into the simulation. NaN means "leave unchanged".
 *
 * @note `update()` is called from the MuJoCo physics thread while the simulation mutex is held.
 * Blocking there stalls both the physics loop and the native viewer, so plugins must not perform
 * blocking operations. Rate-limit expensive work and prefer non-blocking (realtime) publishers.
 */
class MuJoCoROS2ControlPluginBase
{
public:
  virtual ~MuJoCoROS2ControlPluginBase() = default;

  /**
   * @brief Caches the simulation handles and initializes the plugin.
   *
   * Called once by mujoco_ros2_control when the plugin is loaded. Stores @p model and @p data so
   * that `get_mujoco_model()` / `get_sim_data()` work for the lifetime of the plugin, then
   * delegates to `init()`.
   *
   * @param node Shared pointer to the ROS 2 node for accessing parameters
   * @param model Pointer to the live MuJoCo model
   * @param data Pointer to the live MuJoCo data
   * @return whatever `init()` returned: true if initialization was successful
   */
  bool initialize(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
  {
    model_ = model;
    sim_data_ = data;
    return init(node, model_, sim_data_);
  }

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
   * @brief Update the plugin, optionally commanding the simulation.
   *
   * @param control_data Write-only command buffer. Read simulation state via `get_sim_data()`.
   *
   * Only these fields of @p control_data are merged into the simulation, and only where the value
   * is not NaN:
   *
   * - `ctrl[nu]`                actuator controls
   * - `qfrc_applied[nv]`        applied generalized forces
   * - `xfrc_applied[6*nbody]`   applied Cartesian force/torque per body
   * - `qpos[nq]`                generalized positions
   * - `qvel[nv]`               generalized velocities
   *
   * Every other field of @p control_data is unspecified and must not be read.
   *
   * @note NaN means "leave unchanged", so a command persists in the simulation until it is
   * overwritten. To *release* a command, write an explicit value (e.g. `0.0`) rather than leaving
   * the entry as NaN.
   *
   * @note Called once per physics-loop iteration, on the physics thread, with the simulation mutex
   * held, immediately before the simulation is stepped. Writes are therefore visible to the very
   * next step. Avoid blocking operations: they stall physics and the viewer.
   */
  virtual void update(mjData* control_data)
  {
    // Reaching this default body means the plugin only implements the deprecated two-argument
    // overload below. Forward the *live* data rather than control_data: a legacy plugin expects a
    // readable mjData, and because we now run under the simulation mutex its direct writes to the
    // live data are safe and land before the next step.
    if (!legacy_update_warned_)
    {
      legacy_update_warned_ = true;
      RCLCPP_WARN(rclcpp::get_logger("MuJoCoROS2ControlPluginBase"),
                  "Plugin '%s' implements the deprecated update(const mjModel*, mjData*). It is now called from the "
                  "physics thread and still receives the live mjData, so behaviour is unchanged. Please port to "
                  "update(mjData* control_data), reading state via get_sim_data() / get_mujoco_model().",
                  typeid(*this).name());
    }
    (void)control_data;
    update(model_, sim_data_);
  }

  /**
   * @brief Update the plugin (deprecated).
   *
   * @deprecated Implement `update(mjData* control_data)` instead. Plugins that only override this
   * overload keep receiving the live MuJoCo data in @p data and continue to work, but they bypass
   * the NaN-sentinel command buffer and log a deprecation warning on their first update.
   *
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to the live MuJoCo data
   */
  virtual void update(const mjModel* model, mjData* data)
  {
    (void)model;
    (void)data;
  }

  /**
   * @brief Cleanup the plugin
   */
  virtual void cleanup() = 0;

  /**
   * @brief The live MuJoCo model, as cached by `initialize()`.
   */
  const mjModel* get_mujoco_model() const
  {
    return model_;
  }

  /**
   * @brief The live MuJoCo simulation data, as cached by `initialize()`.
   *
   * @note Only safe to dereference from inside `update()`, where the simulation mutex is held by
   * the caller. Reading it from any other thread races the physics loop.
   */
  const mjData* get_sim_data() const
  {
    return sim_data_;
  }

private:
  const mjModel* model_ = nullptr;
  mjData* sim_data_ = nullptr;

  // Latches the one-shot deprecation warning emitted by the default update(mjData*) body.
  bool legacy_update_warned_ = false;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
