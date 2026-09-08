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

#include <limits>
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
 *        (vx, vy, yaw-rate) by writing a hard kinematic override of the base's free-joint
 *        velocity directly into data->qvel every cycle.
 *
 * Wheel-terrain friction/slip modelling is often unreliable enough to make it a poor
 * foundation for testing navigation stacks. This plugin instead subscribes to a
 * cmd_vel-style topic and, every cycle, writes the (optionally clamped) commanded planar
 * velocity directly into the base body's free-joint qvel entries in data. There is no
 * gain to tune and no convergence delay: the measured body velocity on the driven DOFs is
 * exactly the commanded velocity on the very next simulation step.
 *
 * The trade-off is that the driven DOFs cannot be influenced by anything else -- not
 * contacts, not forces from other bodies rigidly mounted on the base (e.g. a swinging
 * arm). They are simply reasserted every cycle regardless of what the physics engine
 * computed in between.
 *
 * Only the planar DOFs are driven: world-frame linear x/y (converted from the commanded
 * body-frame vx/vy via the body's current orientation) and body-local yaw-rate about z.
 * Vertical linear velocity and local roll/pitch are left untouched every cycle, so gravity
 * and contacts continue to settle the base onto the ground normally.
 *
 * Configuration parameters (declared under "mujoco_plugins.<instance_name>.")
 * -------------------------------------------------------------------------------
 *   body                (string, required) - MJCF body name of the base. Must have a
 *                        <freejoint/>; init() fails otherwise, since there is no qvel to
 *                        override without one.
 *   cmd_vel_topic       (string, default "cmd_vel")   - command topic name.
 *   use_stamped_twist   (bool,   default false)       - subscribe to
 *                        geometry_msgs/TwistStamped instead of geometry_msgs/Twist.
 *   max_linear_velocity (double, default +inf) - clamps the commanded planar speed
 *                        sqrt(vx^2+vy^2) [m/s]; direction is preserved when scaled down.
 *   max_yaw_rate        (double, default +inf) - clamps the commanded yaw-rate [rad/s].
 *   cmd_timeout         (double, default 0.5)  - seconds since the last command after
 *                        which it is treated as zero (safety stop).
 */
class BaseVelocityPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  BaseVelocityPlugin() = default;
  ~BaseVelocityPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void pre_step(mjData* data) override;
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

  // Model/body/joint lookup
  const mjModel* model_{ nullptr };
  int body_id_{ -1 };
  int qvel_adr_{ -1 };

  // Clamp parameters
  double max_linear_velocity_{ std::numeric_limits<double>::infinity() };
  double max_yaw_rate_{ std::numeric_limits<double>::infinity() };
  rclcpp::Duration cmd_timeout_{ 0, 0 };

  struct CommandState
  {
    double vx{ 0.0 };
    double vy{ 0.0 };
    double wz{ 0.0 };
    rclcpp::Time time{ 0, 0, RCL_ROS_TIME };
  };

  // Latest command, written by the subscription callback (ROS executor thread) under
  // cmd_mutex_. pre_step() (physics thread) copies it into cached_cmd_ via try_lock, so
  // it never blocks on the subscription callback.
  std::mutex cmd_mutex_;
  CommandState latest_cmd_;

  // Local copy of the command, only ever touched from pre_step() (physics thread).
  CommandState cached_cmd_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__BASE_VELOCITY_PLUGIN_HPP_
