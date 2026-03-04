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

#include "mujoco_ros2_control_plugins/heartbeat_publisher_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool HeartbeatPublisherPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* /*model*/, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  // Create publisher for heartbeat messages
  heartbeat_publisher_ = node_->create_publisher<std_msgs::msg::String>("mujoco_heartbeat", 10);

  // Initialize the last publish time
  last_publish_time_ = node_->get_clock()->now();

  RCLCPP_INFO(logger_,
              "HeartbeatPublisherPlugin initialized. Publishing to topic 'mujoco_heartbeat' every %.3f second(s).",
              publish_period_.seconds());

  return true;
}

void HeartbeatPublisherPlugin::update(const mjModel* /*model*/, mjData* data)
{
  auto current_time = node_->get_clock()->now();
  auto elapsed = current_time - last_publish_time_;

  // Check if it's time to publish
  if (elapsed >= publish_period_)
  {
    auto message = std_msgs::msg::String();
    message.data = "MuJoCo ROS2 Control Heartbeat #" + std::to_string(message_count_) +
                   " | Simulation time: " + std::to_string(data->time) + "s";

    heartbeat_publisher_->publish(message);

    RCLCPP_DEBUG(logger_, "Published heartbeat #%lu at simulation time %.3f", message_count_, data->time);

    message_count_++;
    last_publish_time_ = current_time;
  }
}

void HeartbeatPublisherPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "HeartbeatPublisherPlugin cleanup. Published %lu messages total.", message_count_);

  heartbeat_publisher_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::HeartbeatPublisherPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
