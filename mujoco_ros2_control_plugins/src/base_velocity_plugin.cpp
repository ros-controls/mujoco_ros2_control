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

#include "base_velocity_plugin.hpp"

#include <cmath>
#include <mutex>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

namespace
{
std::string namespacedParamName(const rclcpp::Node::SharedPtr& node, const std::string& name)
{
  const std::string sub_ns = node->get_sub_namespace();
  return sub_ns.empty() ? name : ("mujoco_plugins." + sub_ns + "." + name);
}

// Declares `name` with `default_value` only if it hasn't already been declared
// (e.g. by a test fixture), matching the pattern used by ExternalWrenchPlugin.
template <typename T>
T declareOrGetParameter(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  const std::string full_name = namespacedParamName(node, name);
  if (!node->has_parameter(full_name))
  {
    node->declare_parameter(full_name, default_value);
  }
  return node->get_parameter(full_name).get_value<T>();
}
}  // namespace

bool BaseVelocityPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  model_ = model;

  // "body" has no sane default -- it must name the base's MJCF body.
  const std::string body_name = declareOrGetParameter<std::string>(node_, "body", "");
  if (body_name.empty())
  {
    RCLCPP_ERROR(logger_, "BaseVelocityPlugin requires the 'body' parameter (MJCF body name).");
    return false;
  }

  body_id_ = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
  if (body_id_ < 0)
  {
    RCLCPP_ERROR(logger_, "Body '%s' not found in MuJoCo model.", body_name.c_str());
    return false;
  }

  // A free joint's qvel is what the velocity override writes into -- without one, there is
  // nothing for this plugin to drive, so this is a hard failure rather than a warning.
  for (int j = 0; j < model_->njnt; ++j)
  {
    if (model_->jnt_bodyid[j] == body_id_ && model_->jnt_type[j] == mjJNT_FREE)
    {
      qvel_adr_ = model_->jnt_dofadr[j];
      break;
    }
  }
  if (qvel_adr_ < 0)
  {
    RCLCPP_ERROR(logger_, "Body '%s' has no free joint; BaseVelocityPlugin cannot drive it.", body_name.c_str());
    return false;
  }

  const std::string cmd_vel_topic = declareOrGetParameter<std::string>(node_, "cmd_vel_topic", "cmd_vel");
  const bool use_stamped_twist = declareOrGetParameter<bool>(node_, "use_stamped_twist", false);
  max_linear_velocity_ = declareOrGetParameter<double>(node_, "max_linear_velocity", max_linear_velocity_);
  max_yaw_rate_ = declareOrGetParameter<double>(node_, "max_yaw_rate", max_yaw_rate_);
  const double cmd_timeout_sec = declareOrGetParameter<double>(node_, "cmd_timeout", 0.5);
  cmd_timeout_ = rclcpp::Duration::from_seconds(cmd_timeout_sec);

  if (use_stamped_twist)
  {
    twist_stamped_sub_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::TwistStamped& msg) { twistStampedCallback(msg); });
  }
  else
  {
    twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
                                                                       [this](const geometry_msgs::msg::Twist& msg) {
                                                                         twistCallback(msg);
                                                                       });
  }

  RCLCPP_INFO(logger_,
              "BaseVelocityPlugin initialised for body '%s'. Listening for %s on '%s'. "
              "max_linear_velocity=%.2f max_yaw_rate=%.2f cmd_timeout=%.2fs",
              body_name.c_str(), use_stamped_twist ? "TwistStamped" : "Twist", cmd_vel_topic.c_str(),
              max_linear_velocity_, max_yaw_rate_, cmd_timeout_sec);

  return true;
}

void BaseVelocityPlugin::twistCallback(const geometry_msgs::msg::Twist& msg)
{
  storeCommand(msg.linear.x, msg.linear.y, msg.angular.z);
}

void BaseVelocityPlugin::twistStampedCallback(const geometry_msgs::msg::TwistStamped& msg)
{
  storeCommand(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z);
}

void BaseVelocityPlugin::storeCommand(double vx, double vy, double wz)
{
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  latest_cmd_ = { vx, vy, wz, node_->get_clock()->now() };
}

void BaseVelocityPlugin::pre_step(mjData* data)
{
  // Step 1 - refresh the cached command from the subscription callback without
  // blocking the real-time thread; if the lock is contended, keep using the last
  // successfully cached values.
  if (cmd_mutex_.try_lock())
  {
    cached_cmd_ = latest_cmd_;
    cmd_mutex_.unlock();
  }

  // Step 2 - a stale (or never-received) command is treated as a zero-velocity
  // command, i.e. the base is commanded to stop rather than coast on the last override.
  double vx_cmd = 0.0, vy_cmd = 0.0, wz_cmd = 0.0;
  const rclcpp::Duration age = node_->get_clock()->now() - cached_cmd_.time;
  if (age <= cmd_timeout_)
  {
    vx_cmd = cached_cmd_.vx;
    vy_cmd = cached_cmd_.vy;
    wz_cmd = cached_cmd_.wz;
  }

  // Step 3 - clamp to the configured limits, preserving direction for the planar speed.
  const double linear_speed = std::hypot(vx_cmd, vy_cmd);
  if (linear_speed > max_linear_velocity_)
  {
    const double scale = max_linear_velocity_ / linear_speed;
    vx_cmd *= scale;
    vy_cmd *= scale;
  }
  wz_cmd = mju_clip(wz_cmd, -max_yaw_rate_, max_yaw_rate_);

  // Step 4 - rotate the commanded body-frame linear velocity into the world frame (a free
  // joint's linear qvel is world-frame)
  const mjtNum* xmat = data->xmat + body_id_ * 9;
  data->qvel[qvel_adr_ + 0] = xmat[0] * vx_cmd + xmat[1] * vy_cmd;
  data->qvel[qvel_adr_ + 1] = xmat[3] * vx_cmd + xmat[4] * vy_cmd;
  data->qvel[qvel_adr_ + 5] = wz_cmd;
}

void BaseVelocityPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "BaseVelocityPlugin cleanup.");
  twist_sub_.reset();
  twist_stamped_sub_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::BaseVelocityPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
