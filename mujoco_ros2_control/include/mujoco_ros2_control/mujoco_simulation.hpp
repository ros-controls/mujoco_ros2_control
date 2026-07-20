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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mujoco_ros2_control_msgs/msg/free_joint_state.hpp>
#include <mujoco_ros2_control_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_control_msgs/srv/set_free_joint_state.hpp>
#include <mujoco_ros2_control_msgs/srv/set_pause.hpp>
#include <mujoco_ros2_control_msgs/srv/step_simulation.hpp>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>

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
 * `acquire_data_snapshot()` instead borrows the most recent completed post-step snapshot of
 * `mj_data_`, produced by the physics loop into a separate buffer. Acquiring is a pointer
 * swap under a mutex that is never held for longer than a swap, so the caller performs no
 * scene-sized copy and never waits on physics stepping or refresh. The returned data may
 * lag `mj_data_` by a few timesteps. This is what the hardware interface uses in `write()`.
 *
 * `apply_control_data(...)` will copy control inputs from the provided mjData into staging
 * buffers that the physics loop applies to `mj_data_` immediately before each step.
 * Specifically, it stages `ctrl`, `qfrc_applied`, and `xfrc_applied`. Cartesian forces from
 * `xfrc_applied` compete with inputs from Simulate's drag function, so they are resolved
 * separately. This only takes the control staging mutex and never blocks on physics stepping.
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
   * `control_data` or other containers populated by `copy_mj_data`.
   */
  mjData* data()
  {
    return mj_data_;
  }

  /**
   * @brief Reset simulation state (qpos/qvel/ctrl/sensors/forces) to the captured initial state.
   * @note Caller must hold the sim mutex.
   */
  void reset_world_state(bool fill_initial_state);

  /**
   * @brief Directly set the pose and velocity of one or more free-joint objects, each
   * identified by the name of the body it drives.
   *
   * If any entry fails to resolve, the entire call returns false and no data is modified. 
   * Only once every entry resolves successfully are the poses/velocities written into 
   * `mj_data_->qpos`/`mj_data_->qvel`, followed by a single `mj_forward` call to recompute
   * derived quantities. If the same body name appears more than once, entries are applied 
   * in order, so the last one wins.
   *
   * @param free_joint_states List of free-joint bodies to set. See `FreeJointState.msg` for the
   * per-entry fields (name, pose, twist, reference_frame).
   * @param error_message Set to a human-readable description (identifying the offending entry
   * by index and body name) if this returns false.
   * @return true if every entry's body was found and driven by a free joint, and (if non-empty)
   * every entry's `reference_frame` was found, and the state was applied for all entries; false
   * otherwise (with `error_message` populated and no data modified).
   */
  bool set_free_joint_states(const std::vector<mujoco_ros2_control_msgs::msg::FreeJointState>& free_joint_states,
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
   * Latency-sensitive callers should use `acquire_data_snapshot` or `copy_control_state` instead.
   * @note: If the destination is null it will be created.
   */
  void copy_physics_data(mjData*& destination);

  /**
   * @brief Borrows the latest completed post-step snapshot of `mj_data_` (producer-pays copying).
   *
   * The physics loop fills snapshot buffers on its own thread; this call only swaps pointers
   * to take ownership of the most recent completed one, so the caller never performs a scene-sized
   * copy and never waits for a stepping batch,or in-process refresh. If no new snapshot has
   * completed since the last call, the same buffer is returned again.
   * Also requests a fresh snapshot for the next call.
   *
   * Ownership contract: there is a single borrower slot. The returned buffer remains valid and
   * is never touched by the physics loop until the next `acquire_data_snapshot()` call, at
   * which point the previous buffer is recycled back to the producer. The borrower may freely
   * write to the buffer (e.g., plugins composing control inputs); all such writes are discarded
   * when the buffer is recycled and refilled. Only one consumer may use this API — concurrent
   * callers would swap each other's buffer out from underneath them.
   * Cold-path consumers that need their own copy should use `copy_physics_data` instead.
   */
  mjData* acquire_data_snapshot();

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
   * @brief Stages control fields from `control_data` for the physics loop in a thread safe way.
   *
   * Specifically, copies `control_data->ctrl` and `control_data->qfrc_applied` into staging
   * buffers which the physics loop copies into `mj_data_` immediately before each step.
   * `control_data->xfrc_applied` is copied into `xfrc_plugin_desired_` to avoid conflicts
   * from the simulate app. This does not lock the sim mutex and never waits on stepping.
   */
  void apply_control_data(mjData* control_data);

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
   * @brief Helper function to compose hw interface and simulation provided Cartesian forces.
   *
   * This should be called before stepping the simulation. Assumes the sim mutex is held.
   */
  void apply_staged_control_inputs();

  /**
   * @brief Publishes the per-step control state from `mj_data_`.
   *
   * Called after every mj_step (and forward/reset), before the corresponding /clock tick.
   * Assumes the sim mutex is held; takes the control state mutex.
   */
  void publish_control_state();

  /**
   * @brief Refreshes the consumer-facing snapshot from `mj_data_`.
   *
   * Reclaims the producer-side buffer (clearing snapshot_ready_ if a previous fill was never
   * consumed), fills it outside any shared lock (the sim mutex serializes writers), then
   * raises snapshot_ready_ for the consumer. Assumes the sim mutex is held.
   */
  void refresh_data_snapshot();

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
   * @brief A single free-joint entry that has been fully resolved and is ready to write.
   *
   * Produced by `resolve_free_joint_write` without touching `mj_data_`, so that a whole batch
   * of entries can be validated before any of them are applied (see `set_free_joint_states`).
   */
  struct FreeJointWrite
  {
    int qpos_adr{ -1 };
    int qvel_adr{ -1 };
    mjtNum world_pos[3];
    mjtNum world_quat[4];
    geometry_msgs::msg::Twist twist;
  };

  /**
   * @brief Resolves a single `FreeJointState` entry into a `FreeJointWrite`, without writing
   * anything into `mj_data_`.
   *
   * Looks up `state.name`'s body id, finds the free joint driving it (a body may have at most
   * one), and computes the target world-frame pose -- composing `state.pose` onto
   * `state.reference_frame`'s current world pose if non-empty, or taking it directly as a
   * world-frame pose otherwise.
   *
   * @note Caller must hold the sim mutex, since this reads `mj_model_` and `mj_data_->xpos`/
   * `mj_data_->xquat`.
   * @param state Requested per-body free joint state.
   * @param out Populated with the resolved write on success; left untouched on failure.
   * @param error_message Set to a human-readable description if this returns false.
   * @return true if `state.name` was found and is driven by a free joint, and (if non-empty)
   * `state.reference_frame` was found; false otherwise.
   */
  bool resolve_free_joint_write(const mujoco_ros2_control_msgs::msg::FreeJointState& state, FreeJointWrite& out,
                                std::string& error_message);

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

  // Double-buffered snapshot of mj_data_: the physics loop pays for all scene-sized copies,
  // the consumer borrows the result via acquire_data_snapshot() and performs no copy at all.
  // The producer (any writer holding the sim mutex) fills snapshot_write_ outside any shared
  // lock and raises snapshot_ready_; the consumer, when the flag is up, swaps the two pointers
  // under data_exchange_mutex_ and reads snapshot_read_ in place.
  mjData* snapshot_write_{ nullptr };
  mjData* snapshot_read_{ nullptr };
  bool snapshot_ready_{ false };

  // Control inputs staged by apply_control_data, applied to mj_data_ before each step.
  std::vector<mjtNum> ctrl_staged_;
  std::vector<mjtNum> qfrc_applied_staged_;

  // False until apply_control_data is first called (and cleared on reset), so that initial /
  // reset ctrl values in mj_data_ are not clobbered by stale staging buffers.
  bool control_inputs_staged_{ false };

  // Buffers to track actively applied Cartesian forces from both the plugins and the Simulate /
  // viewer-only drag forces.
  std::vector<mjtNum> xfrc_plugin_desired_;  // Tracks forces from plugins
  std::vector<mjtNum> xfrc_viewer_capture_;  // Tracks forces from the viewer
  std::vector<mjtNum> xfrc_last_written_;    // tracks the last value written to xfrc_applied

  // Guards only the snapshot pointer swap and snapshot_ready_ flag.
  // Lock order: sim_mutex_ (if needed) is always taken before this one.
  std::mutex data_exchange_mutex_;

  // Set by acquire_data_snapshot when a consumer takes the snapshot; the physics loop only
  // runs the expensive full refresh when this is set, so refresh bandwidth tracks consumer
  // demand instead of the batch rate. Starts true so the first refresh happens.
  std::atomic<bool> snapshot_refresh_requested_{ true };

  // Guards the staged control inputs (ctrl_staged_, qfrc_applied_staged_, xfrc_plugin_desired_,
  // control_inputs_staged_). Separate from data_exchange_mutex_ so that staging commands in
  // write() and applying them before each physics step never queue behind a full mjData copy.
  // Critical sections are all small buffer copies.
  // Lock order: sim_mutex_ (if needed) before this one; never held with data_exchange_mutex_.
  std::mutex control_staging_mutex_;

  // Per-step control state served by copy_control_state. Guarded by its own mutex, separate
  // from data_exchange_mutex_, so the reduced control-state copies never queue behind a full
  // mjData snapshot copy.
  // Lock order: sim_mutex_ (if needed) before this one; never held with data_exchange_mutex_.
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
