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

#include <mutex>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

namespace
{
// Declares `name` with `default_value` only if it hasn't already been declared
// (e.g. by a test fixture), matching the pattern used by ExternalWrenchPlugin.
template <typename T>
T declareOrGetParameter(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value)
{
  if (!node->has_parameter(name))
  {
    node->declare_parameter(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
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

  // A free joint is what lets the servo actually move the body; warn (but don't
  // fail) if the body is welded/fixed, since the wrench would then have no effect.
  bool has_free_joint = false;
  for (int j = 0; j < model_->njnt; ++j)
  {
    if (model_->jnt_bodyid[j] == body_id_ && model_->jnt_type[j] == mjJNT_FREE)
    {
      has_free_joint = true;
      break;
    }
  }
  if (!has_free_joint)
  {
    RCLCPP_WARN(logger_,
                "Body '%s' has no free joint; BaseVelocityPlugin's servo wrench will have no effect on it.",
                body_name.c_str());
  }

  const std::string cmd_vel_topic = declareOrGetParameter<std::string>(node_, "cmd_vel_topic", "cmd_vel");
  const bool use_stamped_twist = declareOrGetParameter<bool>(node_, "use_stamped_twist", false);
  kv_linear_ = declareOrGetParameter<double>(node_, "kv_linear", kv_linear_);
  kv_yaw_ = declareOrGetParameter<double>(node_, "kv_yaw", kv_yaw_);
  max_force_ = declareOrGetParameter<double>(node_, "max_force", max_force_);
  max_torque_ = declareOrGetParameter<double>(node_, "max_torque", max_torque_);
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
    twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist& msg) { twistCallback(msg); });
  }

  RCLCPP_INFO(logger_,
              "BaseVelocityPlugin initialised for body '%s'. Listening for %s on '%s'. "
              "kv_linear=%.2f kv_yaw=%.2f cmd_timeout=%.2fs",
              body_name.c_str(), use_stamped_twist ? "TwistStamped" : "Twist", cmd_vel_topic.c_str(), kv_linear_,
              kv_yaw_, cmd_timeout_sec);

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
  cmd_vx_ = vx;
  cmd_vy_ = vy;
  cmd_wz_ = wz;
  last_cmd_time_ = node_->get_clock()->now();
  has_cmd_ = true;
}

void BaseVelocityPlugin::update(const mjModel* /*model_arg*/, mjData* data)
{
  // Step 0 - undo the xfrc_applied contribution written in the previous cycle, so a
  // stale servo force is never left behind (mirrors ExternalWrenchPlugin's pattern).
  // When the system interface also zeroes xfrc_applied before calling update(), this
  // is a harmless no-op on already-zero values.
  if (prev_written_)
  {
    mjtNum* base = data->xfrc_applied + body_id_ * 6;
    for (int j = 0; j < 6; ++j)
    {
      base[j] = 0.0;
    }
    prev_written_ = false;
  }

  if (body_id_ < 0)
  {
    return;
  }

  // Step 1 - refresh the cached command from the subscription callback without
  // blocking the real-time thread; if the lock is contended, keep using the last
  // successfully cached values.
  if (cmd_mutex_.try_lock())
  {
    cached_cmd_vx_ = cmd_vx_;
    cached_cmd_vy_ = cmd_vy_;
    cached_cmd_wz_ = cmd_wz_;
    cached_cmd_time_ = last_cmd_time_;
    cached_has_cmd_ = has_cmd_;
    cmd_mutex_.unlock();
  }

  // Step 2 - a stale (or never-received) command is treated as a zero-velocity
  // command, i.e. the base is commanded to stop rather than coast under whatever
  // wrench was last computed.
  double vx_cmd = 0.0, vy_cmd = 0.0, wz_cmd = 0.0;
  if (cached_has_cmd_)
  {
    const rclcpp::Duration age = node_->get_clock()->now() - cached_cmd_time_;
    if (age <= cmd_timeout_)
    {
      vx_cmd = cached_cmd_vx_;
      vy_cmd = cached_cmd_vy_;
      wz_cmd = cached_cmd_wz_;
    }
  }

  // Step 3 - measure the body's current velocity in its own (body) frame.
  mjtNum vel6[6];  // (rot:lin) = [wx, wy, wz, vx, vy, vz]
  mj_objectVelocity(model_, data, mjOBJ_BODY, body_id_, vel6, /*flg_local=*/1);
  const double vx_meas = vel6[3];
  const double vy_meas = vel6[4];
  const double wz_meas = vel6[2];

  // Step 4 - proportional velocity servo, body frame, saturated.
  const double f_body_x = mju_clip(kv_linear_ * (vx_cmd - vx_meas), -max_force_, max_force_);
  const double f_body_y = mju_clip(kv_linear_ * (vy_cmd - vy_meas), -max_force_, max_force_);
  const double t_body_z = mju_clip(kv_yaw_ * (wz_cmd - wz_meas), -max_torque_, max_torque_);

  // Step 5 - rotate the body-frame force/torque into the world frame. xmat is the
  // row-major 3x3 body->world rotation. Applying the force at the body's centre of
  // mass (i.e. no offset application point) means the torque needs no additional
  // wrench-transport term, unlike ExternalWrenchPlugin's arbitrary application point.
  const mjtNum* xmat = data->xmat + body_id_ * 9;
  const mjtNum force_world[3] = { xmat[0] * f_body_x + xmat[1] * f_body_y, xmat[3] * f_body_x + xmat[4] * f_body_y,
                                  xmat[6] * f_body_x + xmat[7] * f_body_y };
  const mjtNum torque_world[3] = { xmat[2] * t_body_z, xmat[5] * t_body_z, xmat[8] * t_body_z };

  mjtNum* base = data->xfrc_applied + body_id_ * 6;
  for (int j = 0; j < 3; ++j)
  {
    base[j] = force_world[j];
    base[3 + j] = torque_world[j];
  }
  prev_written_ = true;
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
