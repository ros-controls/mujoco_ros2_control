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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__HEARTBEAT_PUBLISHER_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__HEARTBEAT_PUBLISHER_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Simple plugin that publishes a heartbeat message every second
 */
class HeartbeatPublisherPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  HeartbeatPublisherPlugin() = default;
  ~HeartbeatPublisherPlugin() override = default;

  /**
   * @brief Initialize the plugin
   */
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;

  /**
   * @brief Update the plugin (called every simulation step)
   */
  void update(const mjModel* model, mjData* data) override;

  /**
   * @brief Cleanup the plugin
   */
  void cleanup() override;

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_publisher_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("HeartbeatPublisherPlugin");
  rclcpp::Time last_publish_time_;
  rclcpp::Duration publish_period_{ 1, 0 };  // Publish every 1 second
  uint64_t message_count_{ 0 };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__HEARTBEAT_PUBLISHER_PLUGIN_HPP_
