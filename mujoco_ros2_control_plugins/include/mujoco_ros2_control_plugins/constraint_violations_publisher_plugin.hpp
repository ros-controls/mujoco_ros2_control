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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__CONSTRAINT_VIOLATIONS_PUBLISHER_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__CONSTRAINT_VIOLATIONS_PUBLISHER_PLUGIN_HPP_

#include <string>

#include <mujoco_ros2_control_msgs/msg/constraint_violations.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Plugin that reads MuJoCo equality-constraint violations every step and
 *        publishes them as a ConstraintViolations message.
 *
 * Only equality constraints (<equality> in MJCF) are reported.
 * For each active constraint the maximum absolute position-level residual
 * (efc_pos) across all of its constraint rows is published together with a
 * human-readable name.  If the MJCF element has no name attribute the name is
 * synthesised from the connected joints / tendons / bodies:
 *
 *   joint   pair  -> "<joint1>_<joint2>_constraint"
 *   tendon  pair  -> "<tendon1>_<tendon2>_constraint"
 *   weld         -> "<body1>_<body2>_weld_constraint"
 *   connect      -> "<body1>_<body2>_connect_constraint"
 *
 * Topic:   ~/constraint_violations   (relative to the plugin sub-node)
 * Rate:    configurable via ROS parameter "publish_rate" (Hz, default 50 Hz)
 */
class ConstraintViolationsPublisherPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  ConstraintViolationsPublisherPlugin() = default;
  ~ConstraintViolationsPublisherPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  /// Build a human-readable name for equality constraint \p eq_id.
  std::string buildConstraintName(const mjModel* model, int eq_id) const;

  /// Return the type string for equality constraint \p eq_id.
  std::string constraintTypeString(const mjModel* model, int eq_id) const;

  rclcpp::Publisher<mujoco_ros2_control_msgs::msg::ConstraintViolations>::SharedPtr publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<mujoco_ros2_control_msgs::msg::ConstraintViolations>>
      realtime_publisher_;
  mujoco_ros2_control_msgs::msg::ConstraintViolations constraint_violations_msg_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("ConstraintViolationsPublisherPlugin");

  rclcpp::Time last_publish_time_;
  rclcpp::Duration publish_period_{ 0, 20000000 };  // 50 Hz default
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__CONSTRAINT_VIOLATIONS_PUBLISHER_PLUGIN_HPP_
