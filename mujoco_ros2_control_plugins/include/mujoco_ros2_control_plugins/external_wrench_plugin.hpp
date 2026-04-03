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

#include <mujoco_ros2_control_msgs/srv/apply_external_wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Plugin that applies an external wrench (force + torque) to a named
 *        MuJoCo body for a configurable duration.
 *
 * Exposes the ROS 2 service ~/apply_wrench of type
 * mujoco_ros2_control_msgs/srv/ApplyExternalWrench.
 *
 * Service request fields
 * ----------------------
 *   header            – ROS stamp / frame_id (informational)
 *   link_name         – Name of the MuJoCo body (must match MJCF body name)
 *   wrench.force      – Linear force  [N]   expressed in **marker_frame_id frame** (default: base_link)
 *   wrench.torque     – Angular moment [N·m] expressed in **marker_frame_id frame** (default: base_link)
 *   application_point – Point of force application expressed in **marker_frame_id frame**
 *                       (relative to that frame's origin, metres).
 *                       A zero vector applies at the frame origin.
 *   duration          – How long the wrench remains active.
 *                       A zero duration applies it for one simulation step.
 *
 * Service response fields
 * -----------------------
 *   success  – false if the body name was not found in the model.
 *   message  – human-readable status / error description.
 *
 * Multiple concurrent wrenches on different (or the same) bodies are
 * supported.  Each wrench is independent and expires at its own end-time.
 *
 * Implementation notes
 * --------------------
 * Forces are accumulated via mj_applyFT() into data->qfrc_applied.  To avoid
 * permanently contaminating that array the plugin tracks its own contribution
 * from the previous step and subtracts it before re-applying only the
 * currently-active wrenches.
 */
class ExternalWrenchPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  ExternalWrenchPlugin() = default;
  ~ExternalWrenchPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using ApplyExternalWrench = mujoco_ros2_control_msgs::srv::ApplyExternalWrench;

  /// Internal representation of a single active wrench request.
  struct ActiveWrench
  {
    int body_id{ -1 };
    mjtNum force[3]{ 0.0, 0.0, 0.0 };
    mjtNum torque[3]{ 0.0, 0.0, 0.0 };
    mjtNum application_point[3]{ 0.0, 0.0, 0.0 };  ///< body-local frame
    rclcpp::Time end_time{ 0, 0, RCL_ROS_TIME };
    rclcpp::Duration ramp_down_duration{ 0, 0 };
    double current_scale{ 1.0 };  // to be used by the markers for publishing
  };

  /// Service callback — runs in a ROS executor thread.
  void handleApplyWrench(const ApplyExternalWrench::Request::SharedPtr request,
                         ApplyExternalWrench::Response::SharedPtr response);

  /// Publish RViz arrow markers for all active wrenches. Called from update().
  void publishMarkers();

  // ── ROS interfaces ────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("ExternalWrenchPlugin") };
  rclcpp::Service<ApplyExternalWrench>::SharedPtr service_;

  using MarkerArray = visualization_msgs::msg::MarkerArray;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<MarkerArray>> marker_pub_;

  // ── Model pointer (const, valid for simulation lifetime) ─────────────────
  const mjModel* model_{ nullptr };

  // ── Pending queue written by service callback, drained in update() ───────
  std::mutex pending_mutex_;
  std::queue<ActiveWrench> pending_wrenches_;

  // ── Active wrenches — only modified inside update() ───────────────────────
  std::vector<ActiveWrench> active_wrenches_;

  // ── qfrc_applied bookkeeping ──────────────────────────────────────────────
  int nv_{ 0 };
  std::vector<mjtNum> qfrc_temp_;               ///< scratch buffer (nv elements)
  std::vector<mjtNum> qfrc_prev_contribution_;  ///< our last delta on qfrc_applied

  // ── Marker visualization scaling ──────────────────────────────────────────
  /// Arrow length per unit force [m/N]. Parameter: "force_arrow_scale".
  double force_arrow_scale_{ 0.01 };
  /// Arrow length per unit torque [m/(N·m)]. Parameter: "torque_arrow_scale".
  double torque_arrow_scale_{ 0.1 };
  /// TF frame in which markers are published and in which the service caller
  /// must express application_point, force, and torque. Parameter: "marker_frame_id".
  std::string marker_frame_id_{ "base_link" };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__EXTERNAL_WRENCH_PLUGIN_HPP_
