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

#include "mujoco_ros2_control_plugins/external_wrench_plugin.hpp"

#include <algorithm>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace mujoco_ros2_control_plugins
{

bool ExternalWrenchPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  model_ = model;
  nv_ = model->nv;

  qfrc_temp_.assign(nv_, 0.0);
  qfrc_prev_contribution_.assign(nv_, 0.0);

  // Visualization parameters
  if (!node_->has_parameter("force_arrow_scale"))
  {
    node_->declare_parameter("force_arrow_scale", force_arrow_scale_);
  }
  if (!node_->has_parameter("torque_arrow_scale"))
  {
    node_->declare_parameter("torque_arrow_scale", torque_arrow_scale_);
  }
  if (!node_->has_parameter("marker_frame_id"))
  {
    node_->declare_parameter("marker_frame_id", marker_frame_id_);
  }
  force_arrow_scale_ = node_->get_parameter("force_arrow_scale").as_double();
  torque_arrow_scale_ = node_->get_parameter("torque_arrow_scale").as_double();
  marker_frame_id_ = node_->get_parameter("marker_frame_id").as_string();

  marker_pub_raw_ = node_->create_publisher<MarkerArray>("~/wrench_markers", rclcpp::SystemDefaultsQoS());
  marker_pub_ = std::make_unique<realtime_tools::RealtimePublisher<MarkerArray>>(marker_pub_raw_);

  service_ = node_->create_service<ApplyExternalWrench>("apply_wrench",
                                                        std::bind(&ExternalWrenchPlugin::handleApplyWrench, this,
                                                                  std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node_->get_logger(), "ExternalWrenchPlugin initialised. Service available at '%s'.",
              service_->get_service_name());

  return true;
}

void ExternalWrenchPlugin::update(const mjModel* model, mjData* data)
{
  // Step 1 – undo our previous contribution so other plugins / controllers
  //          that also write qfrc_applied are not permanently affected.
  for (int i = 0; i < nv_; ++i)
  {
    data->qfrc_applied[i] -= qfrc_prev_contribution_[i];
  }
  std::fill(qfrc_prev_contribution_.begin(), qfrc_prev_contribution_.end(), 0.0);

  // Step 2 – drain pending wrenches queued by the service callback.
  //          Use try_lock to stay non-blocking in the real-time thread.
  if (pending_mutex_.try_lock())
  {
    while (!pending_wrenches_.empty())
    {
      active_wrenches_.push_back(pending_wrenches_.front());
      pending_wrenches_.pop();
    }
    pending_mutex_.unlock();
  }

  if (active_wrenches_.empty())
  {
    publishMarkers();
    return;
  }

  // Step 3 – apply each active wrench into a scratch buffer so we can
  //          track exactly what we contribute to qfrc_applied.
  std::fill(qfrc_temp_.begin(), qfrc_temp_.end(), 0.0);

  for (const auto& w : active_wrenches_)
  {
    // Transform application_point from body-local frame to world frame.
    //   point_world = xpos + xmat * point_local
    // MuJoCo stores xmat as a row-major 3×3 rotation matrix.
    const mjtNum* xpos = data->xpos + w.body_id * 3;
    const mjtNum* xmat = data->xmat + w.body_id * 9;

    const mjtNum point_world[3] = {
      xpos[0] + xmat[0] * w.application_point[0] + xmat[1] * w.application_point[1] + xmat[2] * w.application_point[2],
      xpos[1] + xmat[3] * w.application_point[0] + xmat[4] * w.application_point[1] + xmat[5] * w.application_point[2],
      xpos[2] + xmat[6] * w.application_point[0] + xmat[7] * w.application_point[1] + xmat[8] * w.application_point[2]
    };

    // mj_applyFT computes the generalised force from a Cartesian force+torque
    // applied at point_world on body w.body_id, and adds the result to the
    // supplied qfrc_target array (our scratch buffer).
    mj_applyFT(model, data, w.force, w.torque, point_world, w.body_id, qfrc_temp_.data());
  }

  // Step 3 (cont.) – add our computed contribution to qfrc_applied and save it
  //                  so we can undo it at the next step.
  for (int i = 0; i < nv_; ++i)
  {
    data->qfrc_applied[i] += qfrc_temp_[i];
  }
  qfrc_prev_contribution_ = qfrc_temp_;

  // Step 4 – remove expired wrenches.  Expiry is checked AFTER applying so
  //          that zero-duration wrenches are applied for exactly one simulation
  //          step before being discarded (as documented in the header).
  const rclcpp::Time now = node_->get_clock()->now();
  active_wrenches_.erase(std::remove_if(active_wrenches_.begin(), active_wrenches_.end(),
                                        [&now](const ActiveWrench& w) { return now >= w.end_time; }),
                         active_wrenches_.end());

  // Step 5 – publish RViz markers for all currently active (non-expired) wrenches.
  publishMarkers();
}

void ExternalWrenchPlugin::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "ExternalWrenchPlugin cleanup.");
  service_.reset();
  marker_pub_.reset();
  marker_pub_raw_.reset();
  node_.reset();
}

// ---------------------------------------------------------------------------
// Marker visualization
// ---------------------------------------------------------------------------

void ExternalWrenchPlugin::publishMarkers()
{
  if (!marker_pub_->trylock())
  {
    return;
  }

  auto& msg = marker_pub_->msg_;
  msg.markers.clear();

  if (active_wrenches_.empty())
  {
    // Clear any markers that may still be displayed in RViz.
    visualization_msgs::msg::Marker del;
    del.header.frame_id = marker_frame_id_;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    msg.markers.push_back(del);
    marker_pub_->unlockAndPublish();
    return;
  }

  const auto stamp = node_->get_clock()->now();

  // Shaft diameter and arrowhead diameter for ARROW markers defined by points.
  static constexpr double kShaftDiam = 0.02;
  static constexpr double kHeadDiam = 0.04;

  int id = 0;
  for (const auto& w : active_wrenches_)
  {
    // Use the application_point and force/torque exactly as supplied in the
    // service request — no MuJoCo feedback lookup needed.
    const double px = w.application_point[0];
    const double py = w.application_point[1];
    const double pz = w.application_point[2];

    // Force arrow (red) — skip if negligible.
    const double f_sq = w.force[0] * w.force[0] + w.force[1] * w.force[1] + w.force[2] * w.force[2];
    if (f_sq > 1e-18)
    {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = marker_frame_id_;
      m.ns = "force";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start, end;
      start.x = px;
      start.y = py;
      start.z = pz;
      end.x = px + w.force[0] * force_arrow_scale_;
      end.y = py + w.force[1] * force_arrow_scale_;
      end.z = pz + w.force[2] * force_arrow_scale_;
      m.points = { start, end };

      m.scale.x = kShaftDiam;  // shaft diameter
      m.scale.y = kHeadDiam;   // head diameter
      m.scale.z = 0.0;         // head length (auto)

      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 0.8f;
      msg.markers.push_back(m);
    }

    // Torque arrow (cyan) — skip if negligible.
    const double t_sq = w.torque[0] * w.torque[0] + w.torque[1] * w.torque[1] + w.torque[2] * w.torque[2];
    if (t_sq > 1e-18)
    {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = marker_frame_id_;
      m.ns = "torque";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point start, end;
      start.x = px;
      start.y = py;
      start.z = pz;
      end.x = px + w.torque[0] * torque_arrow_scale_;
      end.y = py + w.torque[1] * torque_arrow_scale_;
      end.z = pz + w.torque[2] * torque_arrow_scale_;
      m.points = { start, end };

      m.scale.x = kShaftDiam;
      m.scale.y = kHeadDiam;
      m.scale.z = 0.0;

      m.color.r = 0.0f;
      m.color.g = 0.7f;
      m.color.b = 0.9f;
      m.color.a = 0.8f;
      msg.markers.push_back(m);
    }
  }

  marker_pub_->unlockAndPublish();
}

// ---------------------------------------------------------------------------
// Service callback
// ---------------------------------------------------------------------------

void ExternalWrenchPlugin::handleApplyWrench(const ApplyExternalWrench::Request::SharedPtr request,
                                             ApplyExternalWrench::Response::SharedPtr response)
{
  // Validate body name against the cached model pointer.
  const int body_id = mj_name2id(model_, mjOBJ_BODY, request->link_name.c_str());
  if (body_id < 0)
  {
    response->success = false;
    response->message = "Body '" + request->link_name + "' not found in MuJoCo model.";
    RCLCPP_WARN(logger_, "%s", response->message.c_str());
    return;
  }

  ActiveWrench w;
  w.body_id = body_id;
  w.force[0] = request->wrench.force.x;
  w.force[1] = request->wrench.force.y;
  w.force[2] = request->wrench.force.z;
  w.torque[0] = request->wrench.torque.x;
  w.torque[1] = request->wrench.torque.y;
  w.torque[2] = request->wrench.torque.z;
  w.application_point[0] = request->application_point.x;
  w.application_point[1] = request->application_point.y;
  w.application_point[2] = request->application_point.z;

  const rclcpp::Duration duration(request->duration.sec, request->duration.nanosec);
  w.end_time = node_->get_clock()->now() + duration;

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_wrenches_.push(w);
  }

  RCLCPP_INFO(logger_,
              "Applying wrench on body '%s' (id=%d) for %.3f s. "
              "Force [%.2f, %.2f, %.2f] N, Torque [%.2f, %.2f, %.2f] N·m, "
              "Application point [%.3f, %.3f, %.3f] m (body frame).",
              request->link_name.c_str(), body_id, duration.seconds(), w.force[0], w.force[1], w.force[2], w.torque[0],
              w.torque[1], w.torque[2], w.application_point[0], w.application_point[1], w.application_point[2]);

  // Block the service thread until the wrench duration has elapsed so that
  // the response is sent only after the force has been fully applied.
  // Zero-duration wrenches apply for a single simulation step; no sleep needed.
  if (duration.nanoseconds() > 0)
  {
    rclcpp::sleep_for(duration.to_chrono<std::chrono::nanoseconds>());
  }

  response->success = true;
  response->message = "Wrench applied on body '" + request->link_name + "'.";
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ExternalWrenchPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
