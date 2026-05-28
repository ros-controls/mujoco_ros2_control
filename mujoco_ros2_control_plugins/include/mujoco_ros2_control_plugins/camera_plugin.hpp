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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__EXTERNAL_WRENCH_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__EXTERNAL_WRENCH_PLUGIN_HPP_

#include <mutex>
#include <queue>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "sensor_msgs/image_encodings.hpp"

#include <dlfcn.h>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace mujoco_ros2_control_plugins
{

struct CameraData
{
  mjvCamera mjv_cam;
  mjrRect viewport;

  std::string name;
  std::string frame_name;
  std::string info_topic;
  std::string image_topic;
  std::string depth_topic;

  uint32_t width;
  uint32_t height;

  std::vector<uint8_t> image_buffer;
  std::vector<float> depth_buffer;
  std::vector<float> depth_buffer_flipped;

  sensor_msgs::msg::Image image;
  sensor_msgs::msg::Image depth_image;
  sensor_msgs::msg::CameraInfo camera_info;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_pub;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
};

class CameraPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  /** Signature of the GLFW initializer; injectable for testing. */
  using GlfwInitFn = std::function<int()>;

  CameraPlugin() = default;
  ~CameraPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  // ROS interfaces
  rclcpp::Logger logger_{ rclcpp::get_logger("ExternalWrenchPlugin") };

  rclcpp::Publisher<Image>::SharedPtr image_pub_raw_;
  rclcpp::Publisher<Image>::SharedPtr depth_image_pub_raw_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_pub_raw_;

  std::unique_ptr<realtime_tools::RealtimePublisher<Image>> image_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Image>> depth_image_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<CameraInfo>> camera_info_;

  // Model pointer (const, valid for simulation lifetime)
  const mjModel* model_{ nullptr };

  /**
   * @brief Starts the image processing thread in the background.
   *
   * Does nothing when no cameras have been registered. The background thread will initialize
   * its own offscreen GLFW context for rendering images that is separate from the Simulate
   * application, as the context must be created in the running thread.
   *
   * @param glfw_init_fn Callable used to initialize GLFW; defaults to ::glfwInit. Override in
   *                     tests to simulate a display-capable environment without a real GPU.
   */
  void init(GlfwInitFn glfw_init_fn = glfwInit);

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
   * @brief Updates the camera images and publishes info, images, and depth maps.
   */
  void update_cameras();

  rclcpp::Node::SharedPtr node_;

  // Ensures locked access to simulation data for rendering.
  std::recursive_mutex* sim_mutex_{ nullptr };

  mjData* mj_data_;
  mjModel* mj_model_;
  mjData* mj_camera_data_;

  // Image publishing rate
  double camera_publish_rate_;

  // Rendering options for the cameras, currently hard coded to defaults
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  // Containers for camera data and ROS constructs
  std::vector<CameraData> cameras_;

  // Camera processing thread
  std::thread rendering_thread_;
  std::atomic_bool publish_images_;

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

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__EXTERNAL_WRENCH_PLUGIN_HPP_
