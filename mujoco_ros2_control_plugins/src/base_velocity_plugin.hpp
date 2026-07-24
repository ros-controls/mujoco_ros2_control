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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__BASE_VELOCITY_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__BASE_VELOCITY_PLUGIN_HPP_

#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Drives a mobile/floating-base robot from a commanded planar body velocity
 *        (vx, vy, yaw-rate), independent of wheel-ground contact.
 *
 * Wheel-terrain friction/slip modelling is often unreliable enough to make it a poor
 * foundation for testing navigation stacks. This plugin instead subscribes to a
 * cmd_vel-style topic and runs a proportional velocity servo directly on the base
 * body's free joint, applying the result as a world-frame wrench via
 * data->xfrc_applied. Because the servo is force-based (not a kinematic override),
 * the base still collides realistically with walls/obstacles -- only the *propulsion*
 * bypasses wheel-ground contact, not collision response.
 *
 * Only the planar DOFs are driven: body-frame linear x/y and yaw-rate (about body z).
 * Vertical motion and roll/pitch are left entirely to gravity and contacts, so the
 * base settles onto the ground normally.
 *
 * Configuration parameters (declared on the plugin's sub-node)
 * -------------------------------------------------------------
 *   body              (string, required) - MJCF body name of the base. Must have a
 *                      <freejoint/> for the servo to have any effect.
 *   cmd_vel_topic      (string, default "cmd_vel")   - command topic name.
 *   use_stamped_twist  (bool,   default false)       - subscribe to
 *                      geometry_msgs/TwistStamped instead of geometry_msgs/Twist.
 *   kv_linear          (double, default 200.0)       - linear servo gain [N per m/s].
 *   kv_yaw             (double, default 50.0)        - yaw servo gain [N.m per rad/s].
 *   max_force          (double, default 1000.0)      - per-axis linear force cap [N].
 *   max_torque         (double, default 500.0)       - yaw torque cap [N.m].
 *   cmd_timeout        (double, default 0.5)         - seconds since the last command
 *                      after which it is treated as zero (safety stop).
 *
 * Implementation notes
 * ---------------------
 * The servo error is computed in the body frame:
 *
 *   F_body = clamp(kv_linear * (v_cmd - v_meas), +-max_force)   (x, y)
 *   T_body = clamp(kv_yaw    * (w_cmd - w_meas), +-max_torque)  (about z)
 *
 * where v_meas/w_meas come from mj_objectVelocity(..., flg_local=1), i.e. the body's
 * own 6D velocity expressed in its own frame. F_body/T_body are then rotated into the
 * world frame via data->xmat (body -> world rotation) before being written into
 * data->xfrc_applied at the body's slot. Because the force/torque are applied at the
 * body's centre of mass (no offset application point), no wrench-transport term is
 * needed, unlike ExternalWrenchPlugin's arbitrary application point.
 *
 * At the start of every update() the plugin undoes the xfrc_applied contribution it
 * wrote in the previous cycle before writing the new one, mirroring
 * ExternalWrenchPlugin's self-cleanup so the plugin is correct even when called
 * outside of MujocoSystemInterface (which already zeroes xfrc_applied before invoking
 * plugins).
 */
class BaseVelocityPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  BaseVelocityPlugin() = default;
  ~BaseVelocityPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  void twistCallback(const geometry_msgs::msg::Twist& msg);
  void twistStampedCallback(const geometry_msgs::msg::TwistStamped& msg);
  void storeCommand(double vx, double vy, double wz);

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("BaseVelocityPlugin") };
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;

  // Model/body lookup
  const mjModel* model_{ nullptr };
  int body_id_{ -1 };

  // Servo parameters
  double kv_linear_{ 200.0 };
  double kv_yaw_{ 50.0 };
  double max_force_{ 1000.0 };
  double max_torque_{ 500.0 };
  rclcpp::Duration cmd_timeout_{ 0, 0 };

  // Latest command, written by the subscription callback (ROS executor thread) and
  // read by update() (real-time thread) under cmd_mutex_.
  std::mutex cmd_mutex_;
  double cmd_vx_{ 0.0 };
  double cmd_vy_{ 0.0 };
  double cmd_wz_{ 0.0 };
  rclcpp::Time last_cmd_time_{ 0, 0, RCL_ROS_TIME };
  bool has_cmd_{ false };

  // Local cache of the command, only ever touched from update() (single real-time
  // thread), so no lock needed here. Used when a try_lock on cmd_mutex_ fails, so
  // update() never blocks on the subscription callback.
  double cached_cmd_vx_{ 0.0 };
  double cached_cmd_vy_{ 0.0 };
  double cached_cmd_wz_{ 0.0 };
  rclcpp::Time cached_cmd_time_{ 0, 0, RCL_ROS_TIME };
  bool cached_has_cmd_{ false };

  // Whether update() wrote a (possibly zero) contribution to xfrc_applied for
  // body_id_ in the previous cycle, so it can be undone before the next write.
  bool prev_written_{ false };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__BASE_VELOCITY_PLUGIN_HPP_
