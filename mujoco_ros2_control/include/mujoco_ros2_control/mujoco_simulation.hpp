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

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "glfw_adapter.h"  // for mj::GlfwAdapter
#include "simulate.h"      // must be on your include path, handled by CMake

#include <mujoco/mujoco.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mujoco_ros2_control_msgs/msg/free_joint_state.hpp>
#include <mujoco_ros2_control_msgs/msg/simulation_state.hpp>
#include <mujoco_ros2_control_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_control_msgs/srv/set_free_joint_state.hpp>
#include <mujoco_ros2_control_msgs/srv/set_pause.hpp>
#include <mujoco_ros2_control_msgs/srv/step_simulation.hpp>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace mujoco_ros2_control
{

/**
 * @brief ROS 2-based container for the mujoco Simulate application.
 *
 * This class wraps the MuJoCo simulation and Simulate application, while providing necessary
 * hooks to the ros2_control system interface to enable interaction with the sim using
 * "normal" ROS 2 constructs.
 *
 * This class is responsible for mujoco model and data, along with threads for the main physics
 * and rendering loops. It also provides several ROS interfaces for interacting with the
 * underlying simulation - including publishing simulated time to /clock, as well as services
 * for pausing, stepping, and resetting the simulation.
 *
 * Importantly, the physics loop is intended to run at whatever speed (relative realtime) is
 * requested by the user. It is important to not interrupt the loop with locking calls that
 * interact with either the physics sim data, `mj_data_`,  or the model, `mj_model_`.
 * Instead, consumers of this class are provided with functions to read all sim data and provide
 * control inputs from their own mjData containers.
 *
 * The functions relevant to interacting with the physics sim's mjData are:
 *
 * `copy_physics_data(...)` will lock the sim and do a full copy of the existing `mj_data_`
 * into the provided container, which can be used as the caller requires. Because the physics
 * loop can hold the sim mutex for a large fraction of a display refresh while it batches
 * steps, this can block the caller and should not be used from latency-sensitive threads.
 *
 * `copy_control_state(...)` copies the small per-step `ControlState` (positions, velocities,
 * sensor data, ...) that the hardware interface needs every cycle. It is far cheaper than a
 * full mjData copy regardless of scene complexity, and is what the hardware interface uses in
 * `read()`.
 *
 * `stage_actuator_commands(...)` copies actuator commands into a staging buffer that the physics
 * loop merges into `mj_data_->ctrl` immediately before each step. Entries that are NaN mean
 * "leave unchanged", so the caller only has to fill the actuators it actually commands. This
 * only takes the control staging mutex and never blocks on physics stepping.
 *
 * `register_plugins(...)` hands over the MuJoCo plugins to be driven by the physics loop. Each
 * plugin is updated once per loop iteration, with the sim mutex held, immediately before the
 * step batch: it reads live state directly and writes commands into a NaN-filled `control_data`
 * buffer, of which only the non-NaN entries are merged into `mj_data_`.
 *
 * `overwrite_physics_data(...)` will completely replace the data for the sim. Should be used
 * with extreme caution.
 *
 * Thread safety is still somewhat messy, as callers are provided with a simulation mutex that
 * locks the model and data while the actual mujoco engine moves the sim forward. Callers
 * need to be wary of locking that mutex external to this class, as it can have significant
 * consequences on the simulation's speed.
 *
 */
class MujocoSimulation
{
public:
  /**
   * @brief Callback invoked when the simulation's state must be reset.
   *
   * Triggered by the ~/reset_world service or by a any other reset detected in the physics
   * loop. The callback will be run with the sim mutex but after all data has been restored.
   *
   * @param fill_initial_state When true, the caller has not already populated
   *        mj_data_->qpos/qvel/ctrl from a keyframe and the callback should restore the captured
   *        initial state. When false, a keyframe has already been applied.
   */
  using ResetCallback = std::function<void(bool fill_initial_state)>;

  /**
   * @brief Construct a new Mujoco Simulation object. This is a no-op until initialization.
   */
  MujocoSimulation() = default;

  ~MujocoSimulation();

  /**
   * @brief Construct the Simulate application and start the UI thread (if not headless).
   *
   * This initializes the Simulate app and starts the UI thread in the background (if not
   * running headless). It also sets up required publishers and services using the provided node.
   */
  bool initialize(rclcpp::Node::SharedPtr node, const std::string& model_path, const std::string& mujoco_model_topic,
                  double sim_speed_factor, bool headless);

  /**
   * @brief Apply a keyframe to the simulation by name.
   *
   * This locks the simulation mutex and attempts to apply a keyframe by name by calling
   * `mj_resetDataKeyframe`.
   */
  bool apply_keyframe(const std::string& keyframe_name);

  /**
   * @brief Can be called by consumers of this class to store the current state as the "initial" state.
   *
   * In particular, this persises qpos, qvel, and ctrl vectors from the data and writes them into our
   * 'initial_*' vectors.
   */
  void capture_initial_state();

  /**
   * @brief Register a callback function to be called on `reset_world_state`.
   */
  void set_reset_callback(ResetCallback callback);

  /**
   * @brief Start the physics thread. Must be called after load_model().
   */
  void start_physics_thread();

  /**
   * @brief Stop the physics and UI threads if they are running.
   */
  void shutdown();

  /**
   * @brief Accessor for the mujoco model.
   */
  mjModel* model()
  {
    return mj_model_;
  }

  /**
   * @brief Accessor for the raw mujoco simulation data.
   *
   * Users should generally not interact with the physics sim data excepting during setup and
   * other special circumstances. For "normal" processing it is recommended to use
   * `copy_control_state` (reads) and `stage_actuator_commands` (writes), both of which are safe
   * to call from a control loop.
   */
  mjData* data()
  {
    return mj_data_;
  }

  /**
   * @brief Reset simulation state (qpos/qvel/ctrl/sensors/forces) to the captured initial state.
   *
   * @note Caller must hold the sim mutex.
   */
  void reset_world_state(bool fill_initial_state);

  /**
   * @brief Sets the pose and velocity of one or more free-joint objects, identified by body name.
   *
   * Applied atomically: every entry is validated before anything is written, so a single invalid
   * entry leaves the sim state unchanged. Duplicate body names are applied in order, so the last
   * one wins. See `FreeJointState.msg` for per-entry fields.
   *
   * @param error_message Set to a human-readable description (identifying the offending entry)
   * if this returns false.
   * @return true if every entry was applied; false otherwise, with no data modified.
   */
  bool set_free_joint_states(const std::vector<mujoco_ros2_control_msgs::msg::FreeJointState>& free_joints,
                             std::string& error_message);

  /**
   * @brief Copies `mj_model_` into the provided container in a thread safe way.
   *
   * This locks the sim mutex and will pause the physics loop, so should be used sparingly.
   */
  void copy_physics_model(mjModel*& destination);

  /**
   * @brief Copies the provided mjData into mj_data_ in a thread safe way.
   *
   * @note This will completely overwrite the existing data, use with caution!
   */
  void overwrite_physics_data(mjData* source);

  /**
   * @brief Copies `mj_data_` into the provided container in a thread safe way.
   *
   * This locks the sim mutex and will pause the physics loop, so should be used sparingly.
   * Latency-sensitive callers should use `copy_control_state` instead.
   * @note: If the destination is null it will be created.
   */
  void copy_physics_data(mjData*& destination);

  /**
   * @brief Small snapshot of the state the hardware interface needs every control cycle.
   *
   * This data structure is much smaller than a full mjData copy, so publishing and copying
   * it is also far cheaper, regardless of scene complexity. Refreshed by the physics loop
   * after every step, immediately before that step's /clock tick, so a consumer woken by a
   * clock tick sees state that is synchronous with (or newer than) that tick's sim time.
   */
  struct ControlState
  {
    mjtNum time{ 0.0 };
    std::vector<mjtNum> qpos;
    std::vector<mjtNum> qvel;
    std::vector<mjtNum> act;
    std::vector<mjtNum> qfrc_actuator;
    std::vector<mjtNum> sensordata;
    std::vector<mjtNum> ctrl;
  };

  /**
   * @brief Copies the latest per-step control state into the provided container.
   *
   * The copy is under a dedicated mutex and is significantly faster than
   * physics stepping or full-mjData copies.
   * This is what the hardware interface uses in `read()` and `write()`.
   */
  void copy_control_state(ControlState& destination);

  /**
   * @brief Republishes the per-step control state from the current `mj_data_`.
   *
   * The physics loop does this after every step, so this is only needed when a caller has
   * manipulated `mj_data_` directly (e.g. applying initial positions during setup) and wants the
   * next `copy_control_state` to reflect it without waiting for a step to happen.
   *
   * Takes the sim mutex, so it should not be called from a latency-sensitive thread.
   */
  void refresh_control_state();

  /**
   * @brief Stages actuator commands for the physics loop in a thread safe way.
   *
   * The physics loop merges these into `mj_data_->ctrl` immediately before each step. Entries
   * that are NaN are skipped, meaning "leave this actuator unchanged" -- so a caller only fills
   * the actuators it commands, and passive or uncommanded actuators keep whatever the simulation
   * (or a plugin, or a reset) last set. This does not lock the sim mutex and never waits on
   * stepping.
   *
   * @param ctrl_commands Per-actuator commands, sized `model()->nu`. Shorter or longer vectors
   * are rejected with a warning rather than staged partially.
   */
  void stage_actuator_commands(const std::vector<mjtNum>& ctrl_commands);

  /**
   * @brief Hands the MuJoCo plugins to the physics loop, which drives their `update()`.
   *
   * Each registered plugin is updated once per physics-loop iteration, with the sim mutex held,
   * immediately before the step batch. Plugins read live state through
   * `MuJoCoROS2ControlPluginBase::get_sim_data()` and write commands into a NaN-filled
   * `control_data` buffer; only the non-NaN entries are merged into `mj_data_`.
   *
   * Ownership stays with the caller; this class only borrows the shared pointers. Replaces any
   * previously registered plugins.
   *
   * @note Takes the sim mutex, since plugins may be registered after the physics loop has
   * already started.
   */
  void register_plugins(std::vector<std::shared_ptr<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase>> plugins);

  /**
   * @brief Accessor for the mutex which locks access to the data and model.
   */
  std::recursive_mutex& mutex() const
  {
    RCLCPP_WARN_EXPRESSION(logger_, sim_mutex_ == nullptr, "Sim recursive mutex is still nullptr");
    return *sim_mutex_;
  }

  /**
   * @brief Returns the number of steps takein by the physics simulation.
   *
   * Equivalent to the step counter that is shown in the simulate UI.
   */
  uint64_t step_count() const
  {
    return step_count_.load();
  }

private:
  /**
   * @brief Merges the actuator commands staged by `stage_actuator_commands` into `mj_data_->ctrl`.
   *
   * NaN entries are skipped, so uncommanded actuators keep their current value. This should be
   * called before stepping the simulation. Assumes the sim mutex is held; takes the control
   * staging mutex.
   */
  void apply_staged_control_inputs();

  /**
   * @brief Runs every registered plugin's `update()` and merges what they commanded.
   *
   * Fills the commandable fields of `control_data_` with NaN, calls each plugin in registration
   * order, then copies back only the entries the plugins actually wrote. Plugins read live state
   * directly, so no scene-sized copy is involved.
   *
   * Called once per physics-loop iteration -- not once per `mj_step` -- so that plugins keep
   * running at roughly the outer loop rate even when a single iteration batches many steps.
   *
   * @note Assumes the sim mutex is held. Plugin code therefore runs under that mutex.
   */
  void run_plugin_updates();

  /**
   * @brief Publishes the per-step control state from `mj_data_`.
   *
   * Called after every mj_step (and forward/reset), before the corresponding /clock tick.
   * Assumes the sim mutex is held; takes the control state mutex.
   */
  void publish_control_state();

  /**
   * @brief Loops the physics simulation until asked to terminate.
   */
  void physics_loop();

  /**
   * @brief Publish the current sim time to /clock.
   */
  void publish_clock();

  /**
   * @brief Progresses the simulate windows display if not running headless.
   */
  void update_sim_display();

  /**
   * @brief Reset simulation state, applying joint state overrides on top of the restored state.
   *
   * Private because the overrides are assumed to have already been validated (see
   * `validate_joint_state_overrides` / `validate_free_joint_states`) -- a precondition only
   * in-class callers can satisfy, since the validators are private too. Applying unvalidated
   * overrides writes through unresolved (-1) addresses.
   *
   * @note Caller must hold the sim mutex.
   */
  void reset_world_state(bool fill_initial_state, const mujoco_ros2_control_msgs::msg::SimulationState& state_overrides);

  /**
   * @brief Resolves a `header.frame_id` to the body it names: -1 for the world frame (empty
   * string) or for an unknown body.
   *
   * This is the single definition of the frame_id convention; `validate_frame_id` wraps it to
   * turn the unknown-body case into an error message.
   */
  int frame_body_id(const std::string& frame_id) const;

  /**
   * @brief Checks that `frame_id` is empty (world frame) or names a known MuJoCo body.
   *
   * @param field_label Identifies which field ("pose"/"twist") in `error_message` on failure.
   * @return true if valid; false otherwise, with `error_message` set.
   */
  bool validate_frame_id(const std::string& frame_id, const std::string& field_label, std::string& error_message) const;

  /**
   * @brief Returns the id of the free joint driving `body_id`, or -1 if there is none.
   */
  int find_free_joint_id(int body_id) const;

  /**
   * @brief Validates a list of `FreeJointState` entries against the model, without touching
   * `mj_data_`: every entry must name a body driven by a free joint, and both `header.frame_id`s
   * must be empty (world frame) or name a known body.
   *
   * @note Caller must hold the sim mutex.
   * @return true if every entry is valid; false otherwise, with `error_message` set.
   */
  bool validate_free_joint_states(const std::vector<mujoco_ros2_control_msgs::msg::FreeJointState>& free_joints,
                                  std::string& error_message);

  /**
   * @brief Writes free-joint states into `mj_data_->qpos`/`qvel`, resolving each entry's `pose`
   * and `twist` against their own `header.frame_id` -- they may reference different bodies, or
   * vary between world vs. relative frame to another body.
   * Does not republish the control state or run forward dynamics; the caller is responsible for both.
   *
   * @note Caller must hold the sim mutex and have validated `free_joints` first.
   */
  void apply_free_joint_states(const std::vector<mujoco_ros2_control_msgs::msg::FreeJointState>& free_joints);

  /**
   * @brief Validates a `JointState` message against the model, without touching `mj_data_`.
   *
   * Every named joint must be a single-DOF (hinge or slide) MuJoCo joint, and `position` /
   * `velocity` must each be empty or the same length as `name`. `effort` is not supported and
   * must be empty.
   *
   * @note Caller must hold the sim mutex.
   * @return true if the message is valid; false otherwise, with `error_message` set.
   */
  bool validate_joint_state_overrides(const sensor_msgs::msg::JointState& joint_state, std::string& error_message);

  /**
   * @brief Writes single-DOF joint states into `mj_data_->qpos`/`qvel`.
   * Does not republish the control state or run forward dynamics; the caller is responsible for both.
   *
   * @note Caller must hold the sim mutex and have validated `joint_state` first.
   */
  void apply_joint_state_overrides(const sensor_msgs::msg::JointState& joint_state);

  // Service callbacks
  void reset_world_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetWorld::Request> request,
                            std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetWorld::Response> response);
  void set_pause_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::SetPause::Request> request,
                          std::shared_ptr<mujoco_ros2_control_msgs::srv::SetPause::Response> response);
  void step_simulation_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::StepSimulation::Request> request,
                                std::shared_ptr<mujoco_ros2_control_msgs::srv::StepSimulation::Response> response);
  void
  set_free_joint_state_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::SetFreeJointState::Request> request,
                                std::shared_ptr<mujoco_ros2_control_msgs::srv::SetFreeJointState::Response> response);

  rclcpp::Logger get_logger() const
  {
    return logger_;
  }

  // Logger
  rclcpp::Logger logger_ = rclcpp::get_logger("MujocoSimulation");

  // ROS node (owned by the HW interface, used here for services and clock publisher).
  rclcpp::Node::SharedPtr node_;

  // System information
  std::string model_path_;
  std::string mujoco_model_topic_;

  // MuJoCo data pointers, these are the primary containers used by the physics simulation.
  // It is generally not recommended to interact with them directly.
  mjModel* mj_model_{ nullptr };

  // Primary data container for the physics loop. We do not recommend interacting with this
  // directly unless you are sure of what you are doing.
  mjData* mj_data_{ nullptr };

  // Write-only command buffer handed to plugins in run_plugin_updates(). Its commandable fields
  // are NaN-filled before every plugin pass, and only the non-NaN entries are merged back into
  // mj_data_. Nothing else in this container is meaningful.
  mjData* control_data_{ nullptr };

  // MuJoCo plugins driven by the physics loop. Borrowed; owned by the hardware interface.
  std::vector<std::shared_ptr<mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase>> plugins_;

  // Actuator commands staged by stage_actuator_commands, merged into mj_data_->ctrl before each
  // step. NaN entries mean "leave unchanged", which is what keeps initial / reset / plugin-set
  // ctrl values from being clobbered by actuators the hardware interface does not command.
  std::vector<mjtNum> ctrl_staged_;

  // Guards the staged actuator commands (ctrl_staged_). Separate from the sim mutex so that
  // staging commands in write() and merging them before each physics step never queue behind
  // physics stepping. Critical sections are all small buffer copies.
  // Lock order: sim_mutex_ (if needed) before this one.
  std::mutex control_staging_mutex_;

  // Per-step control state served by copy_control_state. Guarded by its own mutex so the reduced
  // control-state copies never queue behind physics stepping.
  // Lock order: sim_mutex_ (if needed) before this one.
  ControlState control_state_;
  std::mutex control_state_mutex_;

  // For rendering
  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;

  // Speed scaling parameter. if set to >0 then we ignore the value set in the simulate app and instead
  // attempt to loop at whatever this is set to. If this is <0, then we use the value from the app.
  double sim_speed_factor_{ -1.0 };

  // True when running without a display (no GLFW window)
  bool headless_{ false };

  // Primary simulate object
  std::unique_ptr<mujoco::Simulate> sim_;

  // Threads for rendering physics and the UI simulation
  std::thread physics_thread_;
  std::thread ui_thread_;

  // Primary clock publisher for the world
  std::shared_ptr<rclcpp::Publisher<rosgraph_msgs::msg::Clock>> clock_publisher_;
  realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_realtime_publisher_;

  // Mutex used inside simulate.h for protecting model/data, we keep a reference
  // here to protect access to shared data.
  // TODO: It would be far better to put all relevant data into a single container with accessors
  //       in a common location rather than passing around the raw pointer to the mutex, but it would
  //       require more work to pull it out of simulate.h.
  std::recursive_mutex* sim_mutex_{ nullptr };

  // Reset world service
  rclcpp::CallbackGroup::SharedPtr reset_world_cb_group_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::ResetWorld>::SharedPtr reset_world_service_;

  // Set pause service
  rclcpp::CallbackGroup::SharedPtr set_pause_cb_group_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::SetPause>::SharedPtr set_pause_service_;

  // Step simulation service
  rclcpp::CallbackGroup::SharedPtr step_simulation_cb_group_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::StepSimulation>::SharedPtr step_simulation_service_;

  // Set free joint state service (teleport/reset a free-joint object's pose)
  rclcpp::CallbackGroup::SharedPtr set_free_joint_state_cb_group_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::SetFreeJointState>::SharedPtr set_free_joint_state_service_;

  // Pending steps to execute while paused, and synchronization for blocking callers
  std::atomic<uint32_t> pending_steps_{ 0 };
  std::atomic<bool> step_diverged_{ false };
  std::atomic<bool> steps_interrupted_{ false };
  std::atomic<bool> keyboard_step_requested_{ false };
  std::atomic<uint64_t> step_count_{ 0 };
  std::mutex steps_cv_mutex_;
  std::condition_variable steps_cv_;

  // Storage for initial state (used for reset_world)
  std::vector<mjtNum> initial_qpos_;
  std::vector<mjtNum> initial_qvel_;
  std::vector<mjtNum> initial_ctrl_;

  // Callback into the HW interface to perform component-side reset bookkeeping.
  ResetCallback reset_callback_;
};

}  // namespace mujoco_ros2_control
