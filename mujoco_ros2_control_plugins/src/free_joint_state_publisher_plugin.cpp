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

#include "free_joint_state_publisher_plugin.hpp"

#include <algorithm>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool FreeJointStatePublisherPlugin::collect_free_joint_entries(const std::vector<std::string>& body_names)
{
  entries_.clear();

  // Enumerate every free joint in the model
  std::vector<FreeJointEntry> all_free_joints;
  for (int i = 0; i < model_->njnt; ++i)
  {
    if (model_->jnt_type[i] == mjJNT_FREE)
    {
      FreeJointEntry entry;
      entry.body_id = model_->jnt_bodyid[i];
      const char* name = mj_id2name(model_, mjOBJ_BODY, entry.body_id);
      entry.body_name = name ? name : "";
      entry.qpos_adr = model_->jnt_qposadr[i];
      entry.qvel_adr = model_->jnt_dofadr[i];
      all_free_joints.push_back(entry);
    }
  }

  if (body_names.empty())
  {
    entries_ = std::move(all_free_joints);
    return true;
  }

  entries_.reserve(body_names.size());
  for (const auto& requested_name : body_names)
  {
    const auto it = std::find_if(all_free_joints.begin(), all_free_joints.end(),
                                 [&requested_name](const FreeJointEntry& e) { return e.body_name == requested_name; });
    if (it == all_free_joints.end())
    {
      RCLCPP_ERROR(logger_, "body_names entry '%s' is not a body driven by a free joint in this MuJoCo model.",
                   requested_name.c_str());
      return false;
    }
    entries_.push_back(*it);
  }
  return true;
}

int FreeJointStatePublisherPlugin::resolve_frame_id(const std::string& frame_id) const
{
  if (frame_id.empty())
  {
    return -1;
  }

  const int body_id = mj_name2id(model_, mjOBJ_BODY, frame_id.c_str());
  if (body_id == -1)
  {
    RCLCPP_ERROR(logger_, "Unknown frame_id body name: '%s'. Falling back to the world frame.", frame_id.c_str());
  }
  return body_id;
}

bool FreeJointStatePublisherPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  model_ = model;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  if (!node_->has_parameter("frame_id"))
  {
    node_->declare_parameter("frame_id", std::string(""));
  }
  if (!node_->has_parameter("body_names"))
  {
    node_->declare_parameter("body_names", std::vector<std::string>{});
  }
  if (!node_->has_parameter("topic"))
  {
    node_->declare_parameter("topic", std::string("free_joint_states"));
  }
  if (!node_->has_parameter("publish_rate"))
  {
    node_->declare_parameter("publish_rate", 50.0);
  }

  const std::string frame_id = node_->get_parameter("frame_id").as_string();
  const std::vector<std::string> body_names = node_->get_parameter("body_names").as_string_array();
  const std::string topic = node_->get_parameter("topic").as_string();
  const double publish_rate = node_->get_parameter("publish_rate").as_double();

  if (publish_rate <= 0.0)
  {
    RCLCPP_ERROR(logger_, "publish_rate must be > 0, got %f.", publish_rate);
    return false;
  }

  if (!collect_free_joint_entries(body_names))
  {
    return false;
  }

  // Resolved once here: the reference body's identity never changes
  frame_body_id_ = resolve_frame_id(frame_id);
  effective_frame_id_ = (frame_body_id_ == -1) ? "" : frame_id;

  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate);
  last_publish_time_ = node_->get_clock()->now();

  message_.free_joints.resize(entries_.size());
  for (std::size_t i = 0; i < entries_.size(); ++i)
  {
    message_.free_joints[i].name = entries_[i].body_name;
  }

  publisher_raw_ = node_->create_publisher<FreeJointStateArray>(topic, rclcpp::SystemDefaultsQoS());
  publisher_ = std::make_unique<realtime_tools::RealtimePublisher<FreeJointStateArray>>(publisher_raw_);

  RCLCPP_INFO(logger_,
              "FreeJointStatePublisherPlugin initialized: publishing %zu free-joint bod%s to '%s' at %.1f Hz "
              "in the '%s' frame.",
              entries_.size(), entries_.size() == 1 ? "y" : "ies", publisher_raw_->get_topic_name(), publish_rate,
              effective_frame_id_.empty() ? "world" : effective_frame_id_.c_str());

  return true;
}

void FreeJointStatePublisherPlugin::update(const mjModel* /*model*/, mjData* data)
{
  const rclcpp::Time now = node_->get_clock()->now();
  if (now - last_publish_time_ < publish_period_)
  {
    return;
  }
  last_publish_time_ = now;

  // Pre-compute the reference frame's inverse world pose/orientation once per publish, reused
  // for every body below.
  mjtNum inv_frame_pos[3] = { 0.0, 0.0, 0.0 };
  mjtNum inv_frame_quat[4] = { 1.0, 0.0, 0.0, 0.0 };
  mjtNum neg_frame_quat[4] = { 1.0, 0.0, 0.0, 0.0 };
  if (frame_body_id_ != -1)
  {
    mju_negPose(inv_frame_pos, inv_frame_quat, data->xpos + 3 * frame_body_id_, data->xquat + 4 * frame_body_id_);
    mju_negQuat(neg_frame_quat, data->xquat + 4 * frame_body_id_);
  }

  message_.header.stamp = now;

  for (std::size_t i = 0; i < entries_.size(); ++i)
  {
    const FreeJointEntry& entry = entries_[i];
    auto& out = message_.free_joints[i];

    const mjtNum* world_pos = data->qpos + entry.qpos_adr;
    const mjtNum* world_quat = data->qpos + entry.qpos_adr + 3;
    const mjtNum* world_linvel = data->qvel + entry.qvel_adr;
    const mjtNum* world_angvel = data->qvel + entry.qvel_adr + 3;

    mjtNum rel_pos[3];
    mjtNum rel_quat[4];
    mjtNum rel_linvel[3];
    mjtNum rel_angvel[3];

    if (frame_body_id_ == -1)
    {
      mju_copy3(rel_pos, world_pos);
      mju_copy4(rel_quat, world_quat);
      mju_copy3(rel_linvel, world_linvel);
      mju_copy3(rel_angvel, world_angvel);
    }
    else
    {
      // (world = frame ∘ relative): relative = frame⁻¹ ∘ world.
      mju_mulPose(rel_pos, rel_quat, inv_frame_pos, inv_frame_quat, world_pos, world_quat);
      mju_rotVecQuat(rel_linvel, world_linvel, neg_frame_quat);
      mju_rotVecQuat(rel_angvel, world_angvel, neg_frame_quat);
    }

    out.pose.header.stamp = now;
    out.pose.header.frame_id = effective_frame_id_;
    out.pose.pose.position.x = rel_pos[0];
    out.pose.pose.position.y = rel_pos[1];
    out.pose.pose.position.z = rel_pos[2];
    out.pose.pose.orientation.w = rel_quat[0];
    out.pose.pose.orientation.x = rel_quat[1];
    out.pose.pose.orientation.y = rel_quat[2];
    out.pose.pose.orientation.z = rel_quat[3];

    out.twist.header.stamp = now;
    out.twist.header.frame_id = effective_frame_id_;
    out.twist.twist.linear.x = rel_linvel[0];
    out.twist.twist.linear.y = rel_linvel[1];
    out.twist.twist.linear.z = rel_linvel[2];
    out.twist.twist.angular.x = rel_angvel[0];
    out.twist.twist.angular.y = rel_angvel[1];
    out.twist.twist.angular.z = rel_angvel[2];
  }

#if REALTIME_TOOLS_VERSION_MAJOR > 2
  publisher_->try_publish(message_);
#else
  publisher_->tryPublish(message_);
#endif
}

void FreeJointStatePublisherPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "FreeJointStatePublisherPlugin cleanup.");
  publisher_.reset();
  publisher_raw_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::FreeJointStatePublisherPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
