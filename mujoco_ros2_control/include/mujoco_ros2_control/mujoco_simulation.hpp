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

#include <mujoco/mujoco.h>

#include <mujoco_ros2_control_msgs/srv/reset_world.hpp>
#include <mujoco_ros2_control_msgs/srv/set_pause.hpp>
#include <mujoco_ros2_control_msgs/srv/step_simulation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "glfw_adapter.h"  // for mj::GlfwAdapter
#include "simulate.h"      // must be on your include path, handled by CMake

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
   * Triggered by the ~/reset_world service or by a UI-driven reset detected in the physics
   * loop. The callback runs while the sim mutex is held; it is responsible for any
   * HW-interface-side bookkeeping (PID resets, command/state interface synchronization, etc.).
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
   * @brief Construct the Simulate application and start the UI thread (or HeadlessAdapter).
   *
   * This initializes the Simulate app and starts the UI thread in the background (if not
   * running headless). It also sets up required publishers and services using the provided node.
   */
  bool initialize(rclcpp::Node::SharedPtr node, const std::string& model_path, const std::string& mujoco_model_topic,
                  double sim_speed_factor, bool headless);

  /**
   * @brief Load the MuJoCo model and compile it.
   *
   * Loads the MJCF as a string either from disk, if a path is specified in `model_path`. If
   * no model is found on disk, then we listen for a string topic. This additionally allocates
   * both the data and the control data once the model is loaded.
   */
  bool load_model();

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
  mjModel* model() const
  {
    return mj_model_;
  }

  /**
   * @brief Accessor for the mujoco data.
   */
  mjData* data() const
  {
    return mj_data_;
  }

  /**
   * @brief Accessor for the mujoco control data.
   */
  mjData* control_data() const
  {
    return mj_data_control_;
  }

  /**
   * @brief Accessor for the mutex which locks access to the data and model.
   */
  std::recursive_mutex& mutex() const
  {
    return *sim_mutex_;
  }

  /**
   * @brief Reset simulation state (qpos/qvel/ctrl/sensors/forces) to the captured initial state.
   * @note Caller must hold the sim mutex.
   */
  void reset_world_state(bool fill_initial_state);

  /**
   * @brief Publish the current sim time to /clock.
   */
  void publish_clock();

private:
  void physics_loop();
  void update_sim_display();

  // Service callbacks
  void reset_world_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetWorld::Request> request,
                            std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetWorld::Response> response);
  void set_pause_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::SetPause::Request> request,
                          std::shared_ptr<mujoco_ros2_control_msgs::srv::SetPause::Response> response);
  void step_simulation_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::StepSimulation::Request> request,
                                std::shared_ptr<mujoco_ros2_control_msgs::srv::StepSimulation::Response> response);

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

  // MuJoCo data pointers
  mjModel* mj_model_{ nullptr };
  mjData* mj_data_{ nullptr };
  mjData* mj_data_control_{ nullptr };

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
