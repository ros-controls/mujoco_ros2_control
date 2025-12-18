/**
 * Copyright (c) 2025, United States Government, as represented by the
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
#include <mutex>
#include <thread>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace mujoco_ros2_control
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

/**
 * @brief Wraps Mujoco cameras for publishing ROS 2 RGB-D streams.
 */
class MujocoCameras
{
public:
  /**
   * @brief Constructs a new MujocoCameras wrapper object.
   *
   * @param node Will be used to construct image publishers
   * @param sim_mutex Provides synchronized access to the mujoco_data object for rendering
   * @param mujoco_data Mujoco data for the simulation
   * @param mujoco_model Mujoco model for the simulation
   * @param camera_publish_rate The rate to publish all camera images, for now all images are published at the same rate.
   */
  explicit MujocoCameras(rclcpp::Node::SharedPtr& node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                         mjModel* mujoco_model, double camera_publish_rate);

  /**
   * @brief Starts the image processing thread in the background.
   *
   * The background thread will initialize its own offscreen glfw context for rendering images that is
   * separate from the Simulate applications, as the context must be in the running thread.
   */
  void init();

  /**
   * @brief Stops the camera processing thread and closes the relevant objects, call before shutdown.
   */
  void close();

  /**
   * @brief Parses camera information from the mujoco model.
   */
  void register_cameras(const hardware_interface::HardwareInfo& hardware_info);

private:
  /**
   * @brief Initializes the rendering context and starts processing.
   */
  void update_loop();

  /**
   * @brief Updates the camera images and publishes info, images, and depth maps.
   */
  void update();

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
};

}  // namespace mujoco_ros2_control
