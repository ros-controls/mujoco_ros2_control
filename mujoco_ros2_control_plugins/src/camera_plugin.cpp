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

#include "mujoco_ros2_control_plugins/camera_plugin.hpp"

#include <algorithm>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace mujoco_ros2_control_plugins
{

bool CameraPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  model_ = model;

  if (!node_->has_parameter("camera_publish_rate"))
  {
    node_->declare_parameter("camera_publish_rate", camera_publish_rate_);
  }
  camera_publish_rate_ = node_->get_parameter("camera_publish_rate").as_double();

  image_pub_raw_ = node_->create_publisher<Image>("~/image_raw", rclcpp::SensorDataQoS());
  depth_image_pub_raw_ = node_->create_publisher<Image>("~/aligned_depth_to_color/image_raw", rclcpp::SensorDataQoS());
  camera_info_pub_raw_ = node_->create_publisher<CameraInfo>("~/camera_info", rclcpp::SensorDataQoS());

  image_pub_ = std::make_unique<realtime_tools::RealtimePublisher<Image>>(image_pub_raw_);
  depth_image_pub_ = std::make_unique<realtime_tools::RealtimePublisher<Image>>(depth_image_pub_raw_);
  camera_info_ = std::make_unique<realtime_tools::RealtimePublisher<CameraInfo>>(camera_info_pub_raw_);

  RCLCPP_INFO(node_->get_logger(), "CameraPlugin initialised.");

  return true;
}

void CameraPlugin::update(const mjModel* /*model_arg*/, mjData* data)
{
}

void CameraPlugin::cleanup()
{
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::CameraPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
