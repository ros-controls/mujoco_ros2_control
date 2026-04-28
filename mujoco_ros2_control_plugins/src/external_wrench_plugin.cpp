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

#include <rclcpp/version.h>
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

  // Visualization parameters
  if (!node_->has_parameter("force_arrow_scale"))
  {
    node_->declare_parameter("force_arrow_scale", force_arrow_scale_);
  }
  if (!node_->has_parameter("torque_arrow_scale"))
  {
    node_->declare_parameter("torque_arrow_scale", torque_arrow_scale_);
  }
  force_arrow_scale_ = node_->get_parameter("force_arrow_scale").as_double();
  torque_arrow_scale_ = node_->get_parameter("torque_arrow_scale").as_double();

  marker_pub_raw_ = node_->create_publisher<MarkerArray>("~/wrench_markers", rclcpp::SystemDefaultsQoS());
  marker_pub_ = std::make_unique<realtime_tools::RealtimePublisher<MarkerArray>>(marker_pub_raw_);

  service_ = node_->create_service<ApplyExternalWrench>("apply_wrench",
                                                        std::bind(&ExternalWrenchPlugin::handleApplyWrench, this,
                                                                  std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(node_->get_logger(), "ExternalWrenchPlugin initialised. Service available at '%s'.",
              service_->get_service_name());

  return true;
}

void ExternalWrenchPlugin::update(const mjModel* /*model_arg*/, mjData* data)
{
  // Step 0 - undo the xfrc_applied contributions written in the previous cycle
  // so stale forces are never left in the array when all wrenches have expired.
  // When the system interface also zeroes xfrc_applied before calling update(),
  // this loop is a harmless no-op on already-zero values.
  for (const int body_id : prev_written_body_ids_)
  {
    mjtNum* base = data->xfrc_applied + body_id * 6;
    for (int j = 0; j < 6; ++j)
    {
      base[j] = 0.0;
    }
  }
  prev_written_body_ids_.clear();

  if (!service_requested_.load(std::memory_order_acquire) && active_wrenches_.empty())
  {
    // No active wrenches and no pending service requests.
    return;
  }

  // Step 1 - drain pending wrenches queued by the service callback.
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
    return;
  }

  // Step 2 - accumulate each active wrench into xfrc_applied.
  // Step 0 already cleared our previous contributions, so += is equivalent to a fresh write.
  //
  // xfrc_applied[body*6 .. body*6+5] = (force_world[3], torque_world_at_xipos[3])
  const rclcpp::Time now_apply = node_->get_clock()->now();

  for (auto& w : active_wrenches_)
  {
    // Compute ramp-down scale: linearly ramp from 1 → 0 over the last
    // ramp_down_duration seconds of the wrench lifetime.
    w.current_scale = 1.0;
    if (w.ramp_down_duration.nanoseconds() > 0)
    {
      const rclcpp::Duration remaining = w.end_time - now_apply;
      if (remaining < w.ramp_down_duration)
      {
        w.current_scale = std::max(0.0, remaining.seconds() / w.ramp_down_duration.seconds());
      }
    }

    // Body pose in world frame. xmat is row-major 3×3 (body → world).
    const mjtNum* xpos = data->xpos + w.body_id * 3;
    const mjtNum* xmat = data->xmat + w.body_id * 9;
    const mjtNum* xipos = data->xipos + w.body_id * 3;  // inertial CoM in world frame

    // Transform application_point from body-local to world frame.
    const mjtNum point_world[3] = {
      xpos[0] + xmat[0] * w.application_point[0] + xmat[1] * w.application_point[1] + xmat[2] * w.application_point[2],
      xpos[1] + xmat[3] * w.application_point[0] + xmat[4] * w.application_point[1] + xmat[5] * w.application_point[2],
      xpos[2] + xmat[6] * w.application_point[0] + xmat[7] * w.application_point[1] + xmat[8] * w.application_point[2]
    };

    // Scale and rotate force/torque from body frame to world frame.
    const mjtNum sf[3] = { w.force[0] * w.current_scale, w.force[1] * w.current_scale, w.force[2] * w.current_scale };
    const mjtNum st[3] = { w.torque[0] * w.current_scale, w.torque[1] * w.current_scale, w.torque[2] * w.current_scale };

    const mjtNum force_world[3] = { xmat[0] * sf[0] + xmat[1] * sf[1] + xmat[2] * sf[2],
                                    xmat[3] * sf[0] + xmat[4] * sf[1] + xmat[5] * sf[2],
                                    xmat[6] * sf[0] + xmat[7] * sf[1] + xmat[8] * sf[2] };
    const mjtNum torque_world[3] = { xmat[0] * st[0] + xmat[1] * st[1] + xmat[2] * st[2],
                                     xmat[3] * st[0] + xmat[4] * st[1] + xmat[5] * st[2],
                                     xmat[6] * st[0] + xmat[7] * st[1] + xmat[8] * st[2] };

    // Transport the wrench from application_point to xipos (body CoM).
    //   τ_at_xipos = τ_world + (point_world − xipos) × force_world
    const mjtNum r[3] = { point_world[0] - xipos[0], point_world[1] - xipos[1], point_world[2] - xipos[2] };
    const mjtNum torque_at_xipos[3] = { torque_world[0] + r[1] * force_world[2] - r[2] * force_world[1],
                                        torque_world[1] + r[2] * force_world[0] - r[0] * force_world[2],
                                        torque_world[2] + r[0] * force_world[1] - r[1] * force_world[0] };

    const int base = w.body_id * 6;
    for (int j = 0; j < 3; ++j)
    {
      data->xfrc_applied[base + j] += force_world[j];
      data->xfrc_applied[base + 3 + j] += torque_at_xipos[j];
    }

    // Track which body slots we touched so Step 0 can undo them next cycle.
    if (std::find(prev_written_body_ids_.begin(), prev_written_body_ids_.end(), w.body_id) ==
        prev_written_body_ids_.end())
    {
      prev_written_body_ids_.push_back(w.body_id);
    }
  }

  // Step 3 - remove expired wrenches.  Expiry is checked AFTER applying so
  //          that zero-duration wrenches are applied for exactly one simulation
  //          step before being discarded (as documented in the header).
  const rclcpp::Time now = node_->get_clock()->now();
  active_wrenches_.erase(std::remove_if(active_wrenches_.begin(), active_wrenches_.end(),
                                        [&now](const ActiveWrench& w) { return now >= w.end_time; }),
                         active_wrenches_.end());

  service_requested_.store(!active_wrenches_.empty(), std::memory_order_release);

  // Step 4 - publish RViz markers for all currently active (non-expired) wrenches.
  publish_markers();
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

void ExternalWrenchPlugin::publish_markers(MarkerArray& markers) const
{
  if (active_wrenches_.empty())
  {
    return;
  }

  // Use time 0 so RViz uses the latest available TF transform rather than
  // looking up at the exact publish time, which can cause extrapolation errors.
  const rclcpp::Time stamp(0, 0, node_->get_clock()->get_clock_type());

  // Shaft diameter and arrowhead diameter for ARROW markers defined by points.
  static constexpr double kShaftDiam = 0.02;
  static constexpr double kHeadDiam = 0.04;

  int id = static_cast<int>(markers.markers.size());
  for (const auto& w : active_wrenches_)
  {
    const double px = w.application_point[0];
    const double py = w.application_point[1];
    const double pz = w.application_point[2];

    // Force arrow (red) — skip if negligible.
    const double f_sq = w.force[0] * w.force[0] + w.force[1] * w.force[1] + w.force[2] * w.force[2];
    if (f_sq > 1e-18)
    {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = w.link_name;
      m.ns = "external_wrench/force";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      const double fs = force_arrow_scale_ * w.current_scale;
      geometry_msgs::msg::Point start, end;
      start.x = px;
      start.y = py;
      start.z = pz;
      end.x = px + w.force[0] * fs;
      end.y = py + w.force[1] * fs;
      end.z = pz + w.force[2] * fs;
      m.points = { start, end };

      m.scale.x = kShaftDiam;
      m.scale.y = kHeadDiam;
      m.scale.z = 0.0;
      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 0.8f;
      markers.markers.push_back(m);
    }

    // Torque arrow (cyan) — skip if negligible.
    const double t_sq = w.torque[0] * w.torque[0] + w.torque[1] * w.torque[1] + w.torque[2] * w.torque[2];
    if (t_sq > 1e-18)
    {
      visualization_msgs::msg::Marker m;
      m.header.stamp = stamp;
      m.header.frame_id = w.link_name;
      m.ns = "external_wrench/torque";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;

      const double ts = torque_arrow_scale_ * w.current_scale;
      geometry_msgs::msg::Point start, end;
      start.x = px;
      start.y = py;
      start.z = pz;
      end.x = px + w.torque[0] * ts;
      end.y = py + w.torque[1] * ts;
      end.z = pz + w.torque[2] * ts;
      m.points = { start, end };

      m.scale.x = kShaftDiam;
      m.scale.y = kHeadDiam;
      m.scale.z = 0.0;
      m.color.r = 0.0f;
      m.color.g = 0.7f;
      m.color.b = 0.9f;
      m.color.a = 0.8f;
      markers.markers.push_back(m);
    }
  }
}

void ExternalWrenchPlugin::publish_markers()
{
  MarkerArray msg;
  publish_markers(msg);
#if RCLCPP_VERSION_MAJOR > 16
  marker_pub_->try_publish(msg);
#else
  marker_pub_->tryPublish(msg);
#endif
}

// ---------------------------------------------------------------------------
// Service callback
// ---------------------------------------------------------------------------

void ExternalWrenchPlugin::handleApplyWrench(const ApplyExternalWrench::Request::SharedPtr request,
                                             ApplyExternalWrench::Response::SharedPtr response)
{
  const auto& wrenches = request->wrenches.external_wrenches;

  if (wrenches.empty())
  {
    response->success = true;
    response->message = "No wrenches provided; nothing applied.";
    return;
  }

  // --- Validate all body names before queuing any wrench (atomic apply) ---
  std::vector<int> body_ids;
  body_ids.reserve(wrenches.size());
  for (const auto& ew : wrenches)
  {
    const std::string& link_name = ew.wrench.header.frame_id;
    const int body_id = mj_name2id(model_, mjOBJ_BODY, link_name.c_str());
    if (body_id < 0)
    {
      response->success = false;
      response->message = "Body '" + link_name + "' not found in MuJoCo model.";
      RCLCPP_WARN(logger_, "%s", response->message.c_str());
      return;
    }
    body_ids.push_back(body_id);
  }

  // --- Build ActiveWrench entries and find the maximum duration to sleep ---
  std::vector<ActiveWrench> new_wrenches;
  new_wrenches.reserve(wrenches.size());

  rclcpp::Duration max_duration{ 0, 0 };
  const rclcpp::Time now = node_->get_clock()->now();

  for (std::size_t i = 0; i < wrenches.size(); ++i)
  {
    const auto& ew = wrenches[i];
    const std::string& link_name = ew.wrench.header.frame_id;
    const rclcpp::Duration duration(ew.duration.sec, ew.duration.nanosec);

    ActiveWrench w;
    w.body_id = body_ids[i];
    w.link_name = link_name;
    w.force[0] = ew.wrench.wrench.force.x;
    w.force[1] = ew.wrench.wrench.force.y;
    w.force[2] = ew.wrench.wrench.force.z;
    w.torque[0] = ew.wrench.wrench.torque.x;
    w.torque[1] = ew.wrench.wrench.torque.y;
    w.torque[2] = ew.wrench.wrench.torque.z;
    w.application_point[0] = ew.application_point.x;
    w.application_point[1] = ew.application_point.y;
    w.application_point[2] = ew.application_point.z;
    w.end_time = now + duration;
    w.ramp_down_duration = rclcpp::Duration(ew.ramp_down_duration.sec, ew.ramp_down_duration.nanosec);

    if (duration > max_duration)
    {
      max_duration = duration;
    }

    RCLCPP_INFO(logger_,
                "Scheduling wrench on body '%s' (id=%d) for %.3f s (ramp-down %.3f s). "
                "Force [%.2f, %.2f, %.2f] N, Torque [%.2f, %.2f, %.2f] N·m, "
                "Application point [%.3f, %.3f, %.3f] m (body frame).",
                link_name.c_str(), body_ids[i], duration.seconds(), w.ramp_down_duration.seconds(), w.force[0],
                w.force[1], w.force[2], w.torque[0], w.torque[1], w.torque[2], w.application_point[0],
                w.application_point[1], w.application_point[2]);

    new_wrenches.push_back(std::move(w));
  }

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    for (auto& w : new_wrenches)
    {
      pending_wrenches_.push(std::move(w));
    }
  }

  service_requested_.store(true, std::memory_order_release);

  // Block the service thread until the longest wrench duration has elapsed so
  // that the response is sent only after all forces have been fully applied.
  // Zero-duration wrenches apply for a single simulation step; no sleep needed.
  if (max_duration.nanoseconds() > 0)
  {
    rclcpp::sleep_for(max_duration.to_chrono<std::chrono::nanoseconds>());
  }

  response->success = true;
  response->message = "Applied " + std::to_string(wrenches.size()) + " wrench(es) successfully.";
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ExternalWrenchPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
