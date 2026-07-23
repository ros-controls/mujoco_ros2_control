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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__FREE_JOINT_STATE_PUBLISHER_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__FREE_JOINT_STATE_PUBLISHER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include <mujoco_ros2_control_msgs/msg/free_joint_state_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Plugin that publishes the pose and velocity of every MuJoCo free-joint body (or a
 *        user-selected subset) to a single topic, in a user-selectable reference frame.
 *
 * Publishes mujoco_ros2_control_msgs/msg/FreeJointStateArray on the configurable ``topic``
 * parameter.
 *
 * Parameters
 * ----------
 *   frame_id      - Name of the MuJoCo body every published pose/twist is expressed relative to.
 *                   Empty (the default) means the world frame.
 *   body_names    - Names of the free-joint bodies to publish. Empty (the default) means every
 *                   free-joint body in the model.
 *   topic         - Output topic name. Defaults to "free_joint_states".
 *   publish_rate  - Publish frequency in Hz. Defaults to 50.0.
 *
 * Frame semantics
 * ---------------
 * When `frame_id` names another body, poses are expressed relative to that body's current world
 * pose, and twists are rotated into that body's current world orientation *without* subtracting
 * the reference body's own velocity.
 */
class FreeJointStatePublisherPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  FreeJointStatePublisherPlugin() = default;
  ~FreeJointStatePublisherPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using FreeJointStateArray = mujoco_ros2_control_msgs::msg::FreeJointStateArray;

  /// A single free-joint body cached at init() time.
  struct FreeJointEntry
  {
    int body_id{ -1 };
    std::string body_name;
    int qpos_adr{ -1 };  /// Index into mjData::qpos
    int qvel_adr{ -1 };  /// Index into mjData::qvel
  };

  /// Scans the model for free joints and populates entries_, applying the body_names filter
  /// (if any). Returns false (logging an error) if a requested body name is invalid.
  bool collect_free_joint_entries(const std::vector<std::string>& body_names);

  /// Resolves frame_id to a body id (-1 for world)
  int resolve_frame_id(const std::string& frame_id) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("FreeJointStatePublisherPlugin") };

  const mjModel* model_{ nullptr };

  rclcpp::Publisher<FreeJointStateArray>::SharedPtr publisher_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<FreeJointStateArray>> publisher_;

  std::vector<FreeJointEntry> entries_;

  /// Reference frame body id resolved once in init(); -1 means the world frame.
  int frame_body_id_{ -1 };
  std::string effective_frame_id_;

  rclcpp::Duration publish_period_{ 0, 0 };
  rclcpp::Time last_publish_time_{ 0, 0, RCL_ROS_TIME };

  /// Reused across update() calls to avoid reallocating the entries vector every publish.
  FreeJointStateArray message_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__FREE_JOINT_STATE_PUBLISHER_PLUGIN_HPP_
