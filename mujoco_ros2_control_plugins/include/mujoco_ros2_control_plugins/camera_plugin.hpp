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

#include <algorithm>
#include <atomic>
#include <functional>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "sensor_msgs/image_encodings.hpp"

#include <dlfcn.h>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

enum class CameraType
{
  DISABLED = 0,  // Publishing is disabled.
  STREAMING,     // Publishing is constant at the specified publish rate.
  POLLED         // Publishing happens only when triggered by a service.
};

struct CameraData
{
  CameraType type = CameraType::STREAMING;

  // Set by the trigger service for polled cameras to request a one-shot capture.
  bool poll_requested{ false };

  // Set when this camera is selected to render on the next pass and is waiting
  // for the rendering thread to consume it, then clear it when rendering occurs.
  bool render_pending{ false };

  mjvCamera mjv_cam;
  mjrRect viewport;

  std::string name;
  std::string frame_name;
  std::string info_topic;
  std::string image_topic;
  std::string depth_topic;
  std::string trigger_service_name;

  uint32_t width;
  uint32_t height;

  std::vector<uint8_t> image_buffer;
  std::vector<float> depth_buffer;

  sensor_msgs::msg::Image image;
  sensor_msgs::msg::Image depth_image;
  sensor_msgs::msg::CameraInfo camera_info;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service;
};

/**
 * @brief Plugin that publishes the mujoco camera data to ROS 2 topics.
 *
 * Camera topics can be named in the ROS 2 controls plugins YAML config file
 * and loaded as node parameters.
 * If any of the parameters are not specified, default topic names will be assigned.
 * A user can provide topic names for multiple cameras as long as cameras are not named the same.
 * Name collision is resolved by last name in the yaml file.
 *
 * Plugin Parameters Structure (and default topics)
 * ---------------------------
 *
 * mujoco_camera_plugin:
 *   type: "mujoco_ros2_control_plugins/CameraPlugin"
 *   # All cameras will publish data at the same rate
 *   camera_publish_rate: 6.0
 *   <camera_name>:
 *     camera_type: <camera_type>  # "disabled", "polled", or "streaming"
 *     frame_name: ""
 *     info_topic: <camera_name>/camera_info
 *     image_topic: <camera_name>/color
 *     depth_topic: <camera_name>/depth
 *     trigger_service_name: <camera_name>/trigger
 *
 * Implementation notes
 * --------------------
 * If the camera name, in the parameters, does not match any of the mujoco cameras
 * the data will not be published to ROS topics.
 * If no cameras parameters are given (but mujoco_camera_plugin and its type are declared)
 * the default type (streaming), topics, and frame are used automatically.
 *
 */
class CameraPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  /** Signature of the GLFW initializer; injectable for testing. */
  using GlfwInitFn = std::function<int()>;

  CameraPlugin() = default;
  ~CameraPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data, GlfwInitFn glfw_init_fn);
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

  /**
   * @brief Synchronously run a single render/publish cycle in the calling thread.
   *
   * This mirrors what the rendering thread does on each pass: streaming cameras are
   * always selected, and polled cameras are selected only if their trigger service has
   * been called since the last cycle. It is intended for tests, where it lets the camera
   * selection logic be exercised deterministically without relying on the asynchronous
   * rendering thread or a live GL context.
   */
  void trigger_update();

  /**
   * @brief Whether the rendering thread has successfully initialized its GL context.
   *
   * Primarily useful for tests that need to wait for the asynchronous rendering thread to
   * come up before forcing a render.
   */
  bool is_rendering_available() const
  {
    return publish_images_.load();
  }

private:
  // ROS interfaces
  rclcpp::Logger logger_{ rclcpp::get_logger("CameraPlugin") };

  /**
   * @brief Stops the camera processing thread and closes the relevant objects, call before shutdown.
   */
  void close();

  /**
   * @brief Parses camera information from the mujoco model.
   */
  void register_cameras();

  /**
   * @brief Initializes the rendering context and starts processing.
   */
  void update_loop();

  /**
   * @brief Renders and publishes every camera that is flagged to render.
   */
  void update_cameras();

  /**
   * @brief Renders and publishes data for a specific camera.
   * @param camera The camera data structure.
   * @param stamp Timestamp applied to all messages published this step.
   */
  void render_and_publish_camera(CameraData& camera, const rclcpp::Time& stamp);

  /**
   * @brief Handles a polled camera trigger to render and publish messages.
   * @param camera The camera data structure.
   */
  void handle_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response, const int camera_idx);

  rclcpp::Node::SharedPtr node_;

  // Ensures locked access to simulation data for rendering.
  std::recursive_mutex* sim_mutex_{ nullptr };

  mjData* mj_data_;
  const mjModel* mj_model_;
  mjData* mj_camera_data_;

  // Image publishing rate (applies to streaming cameras only)
  double camera_publish_rate_{ 5.0 };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };

  // Whether any streaming camera is registered. This lets the update loop skip the
  // streaming time check entirely when only polled/disabled cameras exist.
  bool has_streaming_cameras_{ false };

  // Whether at least one polled camera has a pending trigger request.
  std::atomic_bool poll_pending_{ false };

  // Rendering options for the cameras, currently hard coded to defaults
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  // Scene parameters to keep around to avoid recomputing
  float camera_near_distance_;
  float camera_depth_scale_;

  // Containers for camera data and ROS constructs
  std::vector<CameraData> cameras_;

  // List of camera indices to render on a given pass.
  // Reserved once to the camera count so we do not allocate.
  std::vector<size_t> render_indices_;

  // Camera processing thread and objects for syncing
  std::thread rendering_thread_;
  std::atomic_bool publish_images_{ false };
  std::atomic_bool stop_requested_{ false };
  std::mutex data_mutex_;
  std::condition_variable data_cv_;
  bool new_data_{ false };

  // EGL context for headless rendering (used when GLFW is unavailable)
  EGLDisplay egl_display_{ EGL_NO_DISPLAY };
  EGLContext egl_context_{ EGL_NO_CONTEXT };
  EGLSurface egl_surface_{ EGL_NO_SURFACE };
  bool use_egl_{ false };

  /**
   * @brief Initializes EGL context for headless rendering.
   * @return true if EGL initialization succeeded, false otherwise.
   */
  bool init_egl_context();

  /**
   * @brief Cleans up EGL resources.
   */
  void cleanup_egl_context();
};

}  // namespace mujoco_ros2_control_plugins
