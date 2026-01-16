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

#include "mujoco_ros2_control/mujoco_cameras.hpp"
#include "mujoco_ros2_control/utils.hpp"

#include "sensor_msgs/image_encodings.hpp"

using namespace std::chrono_literals;

namespace mujoco_ros2_control
{

MujocoCameras::MujocoCameras(rclcpp::Node::SharedPtr& node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                             mjModel* mujoco_model, double camera_publish_rate)
  : node_(node)
  , sim_mutex_(sim_mutex)
  , mj_data_(mujoco_data)
  , mj_model_(mujoco_model)
  , camera_publish_rate_(camera_publish_rate)
{
}

void MujocoCameras::register_cameras(const hardware_interface::HardwareInfo& hardware_info)
{
  cameras_.resize(0);
  for (auto i = 0; i < mj_model_->ncam; ++i)
  {
    const char* cam_name = mj_model_->names + mj_model_->name_camadr[i];
    const int* cam_resolution = mj_model_->cam_resolution + 2 * i;
    const mjtNum cam_fovy = mj_model_->cam_fovy[i];

    // Construct CameraData wrapper and set defaults
    CameraData camera;
    camera.name = cam_name;
    camera.mjv_cam.type = mjCAMERA_FIXED;
    camera.mjv_cam.fixedcamid = i;
    camera.width = static_cast<uint32_t>(cam_resolution[0]);
    camera.height = static_cast<uint32_t>(cam_resolution[1]);
    camera.viewport = { 0, 0, cam_resolution[0], cam_resolution[1] };

    // If the hardware_info has a camera of the same name then we pull parameters from there.
    const auto camera_info_maybe = get_sensor_from_info(hardware_info, cam_name);
    if (camera_info_maybe.has_value())
    {
      const auto camera_info = camera_info_maybe.value();
      camera.frame_name = camera_info.parameters.at("frame_name");
      camera.info_topic = camera_info.parameters.at("info_topic");
      camera.image_topic = camera_info.parameters.at("image_topic");
      camera.depth_topic = camera_info.parameters.at("depth_topic");
    }
    // Otherwise set default values for the frame and topics.
    else
    {
      camera.frame_name = camera.name + "_frame";
      camera.info_topic = camera.name + "/camera_info";
      camera.image_topic = camera.name + "/color";
      camera.depth_topic = camera.name + "/depth";
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Adding camera: " << cam_name);
    RCLCPP_INFO_STREAM(node_->get_logger(), "    frame_name: " << camera.frame_name);
    RCLCPP_INFO_STREAM(node_->get_logger(), "    info_topic: " << camera.info_topic);
    RCLCPP_INFO_STREAM(node_->get_logger(), "    image_topic: " << camera.image_topic);
    RCLCPP_INFO_STREAM(node_->get_logger(), "    depth_topic: " << camera.depth_topic);

    // Configure publishers
    camera.camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera.info_topic, 1);
    camera.image_pub = node_->create_publisher<sensor_msgs::msg::Image>(camera.image_topic, 1);
    camera.depth_image_pub = node_->create_publisher<sensor_msgs::msg::Image>(camera.depth_topic, 1);

    // Setup containers for color image data
    camera.image.header.frame_id = camera.frame_name;

    const auto image_size = camera.width * camera.height * 3;
    camera.image_buffer.resize(image_size);
    camera.image.data.resize(image_size);
    camera.image.width = camera.width;
    camera.image.height = camera.height;
    camera.image.step = camera.width * 3;
    camera.image.encoding = sensor_msgs::image_encodings::RGB8;

    // Depth image data
    camera.depth_image.header.frame_id = camera.frame_name;
    camera.depth_buffer.resize(camera.width * camera.height);
    camera.depth_buffer_flipped.resize(camera.width * camera.height);
    camera.depth_image.data.resize(camera.width * camera.height * sizeof(float));
    camera.depth_image.width = camera.width;
    camera.depth_image.height = camera.height;
    camera.depth_image.step = camera.width * sizeof(float);
    camera.depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    // Camera info
    camera.camera_info.header.frame_id = camera.frame_name;
    camera.camera_info.width = camera.width;
    camera.camera_info.height = camera.height;
    camera.camera_info.distortion_model = "plumb_bob";
    camera.camera_info.k.fill(0.0);
    camera.camera_info.r.fill(0.0);
    camera.camera_info.p.fill(0.0);
    camera.camera_info.d.resize(5, 0.0);

    double focal_scaling = (1.0 / std::tan((cam_fovy * M_PI / 180.0) / 2.0)) * camera.height / 2.0;
    camera.camera_info.k[0] = camera.camera_info.p[0] = focal_scaling;
    camera.camera_info.k[2] = camera.camera_info.p[2] = static_cast<double>(camera.width) / 2.0;
    camera.camera_info.k[4] = camera.camera_info.p[5] = focal_scaling;
    camera.camera_info.k[5] = camera.camera_info.p[6] = static_cast<double>(camera.height) / 2.0;
    camera.camera_info.k[8] = camera.camera_info.p[10] = 1.0;

    // Add to list of cameras
    cameras_.push_back(camera);
  }
}

void MujocoCameras::init()
{
  // Start the rendering thread process
  if (!glfwInit())
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to initialize GLFW. Disabling camera publishing.");
    publish_images_ = false;
    return;
  }
  publish_images_ = true;
  rendering_thread_ = std::thread(&MujocoCameras::update_loop, this);
}

void MujocoCameras::close()
{
  publish_images_ = false;
  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
  }
}

void MujocoCameras::update_loop()
{
  // We create an offscreen context specific to this process for managing camera rendering.
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(1, 1, "", NULL, NULL);
  glfwMakeContextCurrent(window);

  // Initialization of the context and data structures has to happen in the rendering thread
  RCLCPP_INFO(node_->get_logger(), "Initializing rendering for cameras");
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  // Turn rangefinder rendering off so we don't get rays in camera images
  mjv_opt_.flags[mjtVisFlag::mjVIS_RANGEFINDER] = 0;

  // Initialize data for camera rendering
  mj_camera_data_ = mj_makeData(mj_model_);
  RCLCPP_INFO(node_->get_logger(), "Starting the camera rendering loop, publishing at %f Hz", camera_publish_rate_);

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // Ensure the context will support the largest cameras
  int max_width = 1, max_height = 1;
  for (const auto& cam : cameras_)
  {
    max_width = std::max(max_width, static_cast<int>(cam.width));
    max_height = std::max(max_height, static_cast<int>(cam.height));
  }
  mjr_resizeOffscreen(max_width, max_height, &mjr_con_);
  RCLCPP_INFO(node_->get_logger(), "Resized offscreen buffer to %d x %d", max_width, max_height);

  // TODO: Support per-camera publish rates?
  rclcpp::Rate rate(camera_publish_rate_);
  while (rclcpp::ok() && publish_images_)
  {
    update();
    rate.sleep();
  }

  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  mj_deleteData(mj_camera_data_);
  glfwDestroyWindow(window);
}

void MujocoCameras::update()
{
  // Rendering is done offscreen
  mjr_setBuffer(mjFB_OFFSCREEN, &mjr_con_);

  // Step 1: Lock the sim and copy data for use in all camera rendering.
  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mjv_copyData(mj_camera_data_, mj_model_, mj_data_);
  }

  // Step 2: Render the scene and copy images to relevant camera data containers.
  for (auto& camera : cameras_)
  {
    // Render scene
    mjv_updateScene(mj_model_, mj_camera_data_, &mjv_opt_, NULL, &camera.mjv_cam, mjCAT_ALL, &mjv_scn_);
    mjr_render(camera.viewport, &mjv_scn_, &mjr_con_);

    // Copy image into relevant buffers
    mjr_readPixels(camera.image_buffer.data(), camera.depth_buffer.data(), camera.viewport, &mjr_con_);
  }

  // Step 3: Adjust the images and copy depth data.
  const float near = static_cast<float>(mj_model_->vis.map.znear * mj_model_->stat.extent);
  const float far = static_cast<float>(mj_model_->vis.map.zfar * mj_model_->stat.extent);
  const float depth_scale = 1.0f - near / far;
  for (auto& camera : cameras_)
  {
    // Fix non-linear projections in the depth image and flip the data.
    // https://github.com/google-deepmind/mujoco/blob/3.4.0/python/mujoco/renderer.py#L190
    for (uint32_t h = 0; h < camera.height; h++)
    {
      for (uint32_t w = 0; w < camera.width; w++)
      {
        auto idx = h * camera.width + w;
        auto idx_flipped = (camera.height - 1 - h) * camera.width + w;
        camera.depth_buffer[idx] = near / (1.0f - camera.depth_buffer[idx] * (depth_scale));
        camera.depth_buffer_flipped[idx_flipped] = camera.depth_buffer[idx];
      }
    }
    // Copy flipped data into the depth image message, floats -> unsigned chars
    std::memcpy(&camera.depth_image.data[0], camera.depth_buffer_flipped.data(), camera.depth_image.data.size());

    // OpenGL's coordinate system's origin is in the bottom left, so we invert the images row-by-row
    auto row_size = camera.width * 3;
    for (uint32_t h = 0; h < camera.height; h++)
    {
      auto src_idx = h * row_size;
      auto dest_idx = (camera.height - 1 - h) * row_size;
      std::memcpy(&camera.image.data[dest_idx], &camera.image_buffer[src_idx], row_size);
    }
  }

  // Step 4: Publish the images.
  for (auto& camera : cameras_)
  {
    // Publish images and camera info
    const auto time = node_->now();
    camera.image.header.stamp = time;
    camera.depth_image.header.stamp = time;
    camera.camera_info.header.stamp = time;

    camera.image_pub->publish(camera.image);
    camera.depth_image_pub->publish(camera.depth_image);
    camera.camera_info_pub->publish(camera.camera_info);
  }
}

}  // namespace mujoco_ros2_control
