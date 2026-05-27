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

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

class CameraPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  CameraPlugin() = default;
  ~CameraPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("ExternalWrenchPlugin") };

  double camera_publish_rate_;

  //
  rclcpp::Publisher<Image>::SharedPtr image_pub_raw_;
  rclcpp::Publisher<Image>::SharedPtr depth_image_pub_raw_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_pub_raw_;

  std::unique_ptr<realtime_tools::RealtimePublisher<Image>> image_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<Image>> depth_image_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<CameraInfo>> camera_info_;

  // Model pointer (const, valid for simulation lifetime)
  const mjModel* model_{ nullptr };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__EXTERNAL_WRENCH_PLUGIN_HPP_
