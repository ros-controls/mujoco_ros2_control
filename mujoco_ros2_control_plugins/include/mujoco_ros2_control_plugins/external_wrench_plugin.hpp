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
 * @brief Plugin that applies one or more external wrenches (force + torque) to
 *        named MuJoCo bodies for configurable durations.
 *
 * Exposes the ROS 2 service ~/apply_wrench of type
 * mujoco_ros2_control_msgs/srv/ApplyExternalWrench.
 *
 * The service accepts an array of ExternalWrench messages so that multiple
 * wrenches can be submitted atomically in a single call.  All wrenches are
 * validated before any are applied: if any body name is unknown the entire
 * request is rejected.
 *
 * Each ExternalWrench fields
 * --------------------------
 *   wrench.header.frame_id  - Name of the MuJoCo body (must match MJCF body name)
 *   wrench.wrench.force     - Linear force  [N]   expressed in the **body (link) frame**
 *   wrench.wrench.torque    - Angular moment [N·m] expressed in the **body (link) frame**
 *   application_point       - Point of force application in the **body (link) frame**
 *                             (relative to the link frame origin, metres).
 *                             A zero vector applies at the link frame origin.
 *   duration                - How long the wrench remains active.
 *                             A zero duration applies it for one simulation step.
 *   ramp_down_duration      - Optional linear ramp-down window at the end of duration.
 *
 * Service response fields
 * -----------------------
 *   success  - false if any body name was not found in the model.
 *   message  - human-readable status / error description.
 *
 * Multiple concurrent wrenches on different (or the same) bodies are
 * supported.  Each wrench is independent and expires at its own end-time.
 *
 * Implementation notes
 * --------------------
 * Forces are written into data->xfrc_applied (Cartesian 6D force/torque per
 * body, world frame, applied at the body's inertial CoM xipos).  MuJoCo reads
 * this array during mj_step and both applies the force AND renders it as an
 * arrow in the native viewer when "Perturb forces" is enabled.
 *
 * The service input is expressed in the body's local frame at an arbitrary
 * application point.  The plugin converts this to the world-frame wrench at
 * xipos using the standard wrench-transport formula:
 *
 *   F_world      = R * F_body
 *   τ_world      = R * τ_body
 *   τ_at_xipos   = τ_world + (p_world − xipos) × F_world
 *
 * where R = data->xmat (body → world rotation) and
 *       p_world = data->xpos + R * application_point_body.
 *
 * At the start of every update() call the plugin zeroes the xfrc_applied slots
 * it wrote during the previous cycle before re-accumulating the current active
 * wrenches.  This self-cleanup ensures the plugin leaves no stale contributions
 * in xfrc_applied when all wrenches have expired.  When the surrounding system
 * interface also zeroes xfrc_applied before calling update() (as
 * MujocoSystemInterface::read() does), the plugin's undo step is a harmless
 * no-op on already-zero values.
 */
class ExternalWrenchPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  ExternalWrenchPlugin() = default;
  ~ExternalWrenchPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

  /**
   * @brief Append RViz arrow markers for all currently active wrenches to @p markers.
   *
   * This method is additive: existing entries in @p markers are preserved.
   * It emits nothing when there are no active wrenches, so callers can call it
   * unconditionally and aggregate contributions from multiple plugins before
   * publishing a single MarkerArray.
   *
   * Marker namespaces:
   *   "external_wrench/force"  – red arrows, one per active wrench with non-zero force
   *   "external_wrench/torque" – cyan arrows, one per active wrench with non-zero torque
   *
   * @param markers  MarkerArray to append to.
   */
  void publish_markers(visualization_msgs::msg::MarkerArray& markers) const;

private:
  using ApplyExternalWrench = mujoco_ros2_control_msgs::srv::ApplyExternalWrench;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  /// Internal representation of a single active wrench request.
  struct ActiveWrench
  {
    int body_id{ -1 };
    std::string link_name;
    mjtNum force[3]{ 0.0, 0.0, 0.0 };              ///< body-local frame
    mjtNum torque[3]{ 0.0, 0.0, 0.0 };             ///< body-local frame
    mjtNum application_point[3]{ 0.0, 0.0, 0.0 };  ///< body-local frame
    rclcpp::Time end_time{ 0, 0, RCL_ROS_TIME };
    rclcpp::Duration ramp_down_duration{ 0, 0 };
    double current_scale{ 1.0 };  ///< ramp-down scale, used by markers too
  };

  /// Service callback — runs in a ROS executor thread.
  void handleApplyWrench(const ApplyExternalWrench::Request::SharedPtr request,
                         ApplyExternalWrench::Response::SharedPtr response);

  /// Publish the result of publish_markers() via the realtime publisher. Called from update().
  void publish_markers();

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("ExternalWrenchPlugin") };
  rclcpp::Service<ApplyExternalWrench>::SharedPtr service_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_raw_;
  std::unique_ptr<realtime_tools::RealtimePublisher<MarkerArray>> marker_pub_;

  // Model pointer (const, valid for simulation lifetime)
  const mjModel* model_{ nullptr };

  // Pending queue written by service callback, drained in update()
  std::mutex pending_mutex_;
  std::queue<ActiveWrench> pending_wrenches_;

  // Active wrenches — only modified inside update()
  std::vector<ActiveWrench> active_wrenches_;

  // Body IDs whose xfrc_applied slots were written in the previous update() call.
  // Zeroed at the start of the next update() before re-accumulating.
  std::vector<int> prev_written_body_ids_;

  // Marker visualization scaling
  /// Arrow length per unit force [m/N]. Parameter: "force_arrow_scale".
  double force_arrow_scale_{ 0.01 };
  /// Arrow length per unit torque [m/(N·m)]. Parameter: "torque_arrow_scale".
  double torque_arrow_scale_{ 0.1 };

  std::atomic_bool service_requested_{ false };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__EXTERNAL_WRENCH_PLUGIN_HPP_
