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

#include <cmath>
#include <limits>
#include <string>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control_plugins
{

/**
 * @brief The value an unwritten entry of a command buffer holds: "leave this unchanged".
 *
 * This is the sentinel the whole command path is built on, so refer to it rather than spelling out
 * a NaN literal. Anything else -- including a deliberate 0.0 -- is a command.
 */
inline constexpr mjtNum kUnsetCommand = std::numeric_limits<mjtNum>::quiet_NaN();

/**
 * @brief Whether @p value is a command rather than an unwritten entry.
 */
inline bool is_commanded(mjtNum value)
{
  return !std::isnan(value);
}

/**
 * @brief Marks @p count entries of @p values as unset, i.e. commanding nothing.
 */
inline void mark_unset(mjtNum* values, int count)
{
  mju_fill(values, kUnsetCommand, count);
}

/**
 * @brief Copies the commanded (non-unset) entries of @p commanded over @p destination.
 *
 * Entries left unset are skipped, so whatever is already in @p destination survives.
 */
inline void merge_commands(const mjtNum* commanded, mjtNum* destination, int count)
{
  for (int i = 0; i < count; ++i)
  {
    if (is_commanded(commanded[i]))
    {
      destination[i] = commanded[i];
    }
  }
}

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
 *   buffer is *write-only*: every commandable field arrives filled with `kUnsetCommand`, and only
 *   the entries a plugin actually writes are merged back into the simulation.
 *
 * The commandable fields of `control_data` are:
 *
 * - `ctrl[nu]`                actuator controls
 * - `qfrc_applied[nv]`        applied generalized forces
 * - `xfrc_applied[6*nbody]`   applied Cartesian force/torque per body
 * - `qpos[nq]`                generalized positions
 * - `qvel[nv]`                generalized velocities
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
    // Remembered only so diagnostics can name the plugin the way the user configured it.
    plugin_name_ = node ? node->get_fully_qualified_name() : "<unknown>";
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
   * Only the commandable fields listed in the class documentation are merged into the simulation,
   * and only where the entry is a command rather than `kUnsetCommand`. Every other field of
   * @p control_data is unspecified and must not be read.
   *
   * @note An unset entry means "leave unchanged", so a command persists in the simulation until it
   * is overwritten. To *release* a command, write an explicit value (e.g. `0.0`) rather than
   * leaving the entry unset.
   *
   * @note Called once per physics-loop iteration, on the physics thread, with the simulation mutex
   * held, immediately before the simulation is stepped. Writes are therefore visible to the very
   * next step. Avoid blocking operations: they stall physics and the viewer.
   */
  virtual void update(mjData* /*control_data*/)
  {
    // Reaching this default body means the plugin did not override this overload. It either
    // implements the deprecated one below, or it implements neither -- which we can tell apart
    // because the deprecated overload's own default body raises the flag.
    //
    // Forward the *live* data rather than control_data: a legacy plugin expects a readable mjData,
    // and because we now run under the simulation mutex its direct writes to the live data are
    // safe and land before the next step.
    base_legacy_update_invoked_ = false;
    update(model_, sim_data_);

    if (base_legacy_update_invoked_)
    {
      RCLCPP_ERROR_ONCE(logger(),
                        "Plugin '%s' implements neither update(mjData*) nor the deprecated "
                        "update(const mjModel*, mjData*), so it will never do anything. Check the "
                        "signature -- update(mjData* control_data) is the one to override.",
                        plugin_name_.c_str());
    }
    else if (!legacy_update_warned_)
    {
      legacy_update_warned_ = true;
      RCLCPP_WARN(logger(),
                  "Plugin '%s' implements the deprecated update(const mjModel*, mjData*). It is now called from the "
                  "physics thread and still receives the live mjData, so behaviour is unchanged. Please port to "
                  "update(mjData* control_data), reading state via get_sim_data() / get_mujoco_model().",
                  plugin_name_.c_str());
    }
  }

  /**
   * @brief Update the plugin (deprecated).
   *
   * @deprecated Implement `update(mjData* control_data)` instead. Plugins that only override this
   * overload keep receiving the live MuJoCo data in @p data and continue to work, but they bypass
   * the command buffer and log a deprecation warning on their first update.
   *
   * @param model Pointer to the MuJoCo model
   * @param data Pointer to the live MuJoCo data
   */
  virtual void update(const mjModel* /*model*/, mjData* /*data*/)
  {
    // Only reached when the plugin overrides neither overload; see update(mjData*).
    base_legacy_update_invoked_ = true;
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
  static rclcpp::Logger logger()
  {
    return rclcpp::get_logger("MuJoCoROS2ControlPluginBase");
  }

  const mjModel* model_ = nullptr;
  mjData* sim_data_ = nullptr;

  // Configured name of this plugin, used only to make diagnostics identifiable.
  std::string plugin_name_ = "<uninitialized>";

  // Set by the deprecated overload's default body so update(mjData*) can tell "implements the
  // deprecated overload" apart from "implements neither".
  bool base_legacy_update_invoked_ = false;

  // Latches the one-shot deprecation warning. RCLCPP_WARN_ONCE would not do: its static local is
  // shared by every instance, so only the first legacy plugin in the process would be reported.
  bool legacy_update_warned_ = false;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__PLUGIN_BASE_HPP_
