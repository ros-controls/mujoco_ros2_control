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

#include "mujoco_ros2_control_plugins/constraint_violations_publisher_plugin.hpp"

#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/version.h>
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{
namespace
{

/// Return the name registered with MuJoCo for \p id of \p obj_type,
/// or \p fallback if no name was assigned.
std::string safeObjName(const mjModel* model, int obj_type, int id, const std::string& fallback)
{
  const char* n = mj_id2name(model, obj_type, id);
  return (n && n[0] != '\0') ? std::string(n) : fallback;
}

}  // namespace

bool ConstraintViolationsPublisherPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  // Optional publish-rate parameter (Hz)
  int rate_hz = 50;
  if (!node_->has_parameter("publish_rate"))
  {
    node_->declare_parameter("publish_rate", rate_hz);
  }
  rate_hz = static_cast<int>(node_->get_parameter("publish_rate").as_int());
  if (rate_hz <= 0)
  {
    RCLCPP_WARN(node_->get_logger(), "publish_rate must be > 0, ignoring value %d and using 50 Hz.", rate_hz);
    rate_hz = 50;
  }
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / static_cast<double>(rate_hz));

  publisher_ =
      node_->create_publisher<mujoco_ros2_control_msgs::msg::ConstraintViolations>("constraint_violations", 10);
  realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<mujoco_ros2_control_msgs::msg::ConstraintViolations>>(
          publisher_);

  last_publish_time_ = node_->get_clock()->now();

  // Preallocate message vectors and precompute constraint metadata
  max_constraints_ = model->neq;
  constraint_violations_msg_.names.reserve(max_constraints_);
  constraint_violations_msg_.types.reserve(max_constraints_);
  constraint_violations_msg_.violations.reserve(max_constraints_);

  constraint_metadata_.resize(max_constraints_);
  for (int eq_id = 0; eq_id < max_constraints_; ++eq_id)
  {
    constraint_metadata_[eq_id].name = buildConstraintName(model, eq_id);
    constraint_metadata_[eq_id].type = constraintTypeString(model, eq_id);
  }

  RCLCPP_INFO(node_->get_logger(),
              "ConstraintViolationsPublisherPlugin initialised. "
              "Model has %d equality constraint(s). Publishing at %d Hz on '%s'.",
              model->neq, rate_hz, publisher_->get_topic_name());

  return true;
}

void ConstraintViolationsPublisherPlugin::update(const mjModel* model, mjData* data)
{
  if (max_constraints_ == 0)
  {
    return;  // no equality constraints in the model
  }

  const auto now = node_->get_clock()->now();
  if (now - last_publish_time_ < publish_period_)
  {
    return;
  }
  last_publish_time_ = now;

  // Cache efc pointers from the struct *once* before dereferencing.
  // mj_data_control_ is written by the sim thread (mj_copyData) without
  // read() holding sim_mutex_, so the pointer fields can change concurrently.
  const int nefc = data->nefc;
  const int* efc_type = data->efc_type;
  const int* efc_id = data->efc_id;
  const mjtNum* efc_pos = data->efc_pos;

  if (nefc == 0 || !efc_type || !efc_id || !efc_pos)
  {
    return;
  }

  // Accumulate the maximum absolute position-level residual (efc_pos) for
  // each active equality constraint.  A single equality constraint may span
  // multiple consecutive rows in the efc arrays (e.g. weld = 6 rows).
  // efc_id[i] holds the index into model->eq_* for equality rows.
  std::unordered_map<int, double> max_violation;  // eq_id -> max |efc_pos|

  for (int i = 0; i < nefc; ++i)
  {
    if (efc_type[i] != mjCNSTR_EQUALITY)
    {
      continue;
    }
    const int eq_id = efc_id[i];
    const double viol = std::abs(efc_pos[i]);
    auto it = max_violation.find(eq_id);
    if (it == max_violation.end())
    {
      max_violation[eq_id] = viol;
    }
    else
    {
      it->second = std::max(it->second, viol);
    }
  }

  if (max_violation.empty())
  {
    return;  // nothing active – skip publishing
  }

  if (!realtime_publisher_)
  {
    return;
  }

  constraint_violations_msg_.stamp = now;
  // Resize to actual violation count; vectors were preallocated in init()
  constraint_violations_msg_.names.resize(max_violation.size());
  constraint_violations_msg_.types.resize(max_violation.size());
  constraint_violations_msg_.violations.resize(max_violation.size());

  // Fill data into preallocated vectors using precomputed metadata (name and type are static)
  int idx = 0;
  for (const auto& [eq_id, violation] : max_violation)
  {
    constraint_violations_msg_.names[idx] = constraint_metadata_[eq_id].name;
    constraint_violations_msg_.types[idx] = constraint_metadata_[eq_id].type;
    constraint_violations_msg_.violations[idx] = violation;
    ++idx;
  }

#if RCLCPP_VERSION_MAJOR >= 16
  realtime_publisher_->try_publish(constraint_violations_msg_);
#else
  realtime_publisher_->tryPublish(constraint_violations_msg_);
#endif
}

void ConstraintViolationsPublisherPlugin::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "ConstraintViolationsPublisherPlugin cleanup.");
  realtime_publisher_.reset();
  publisher_.reset();
  node_.reset();
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

std::string ConstraintViolationsPublisherPlugin::buildConstraintName(const mjModel* model, int eq_id) const
{
  // Prefer the name given in the MJCF <equality> element.
  const char* name = mj_id2name(model, mjOBJ_EQUALITY, eq_id);
  if (name && name[0] != '\0')
  {
    return std::string(name);
  }

  // Synthesise a name from the connected objects.
  const int eq_type = model->eq_type[eq_id];
  const int obj1id = model->eq_obj1id[eq_id];
  const int obj2id = model->eq_obj2id[eq_id];

  switch (eq_type)
  {
    case mjEQ_JOINT:
    {
      const std::string n1 = safeObjName(model, mjOBJ_JOINT, obj1id, "joint" + std::to_string(obj1id));
      const std::string n2 = safeObjName(model, mjOBJ_JOINT, obj2id, "joint" + std::to_string(obj2id));
      return n1 + "_" + n2 + "_constraint";
    }
    case mjEQ_TENDON:
    {
      const std::string n1 = safeObjName(model, mjOBJ_TENDON, obj1id, "tendon" + std::to_string(obj1id));
      const std::string n2 = safeObjName(model, mjOBJ_TENDON, obj2id, "tendon" + std::to_string(obj2id));
      return n1 + "_" + n2 + "_constraint";
    }
    case mjEQ_CONNECT:
    {
      // obj1id / obj2id are body IDs; body 0 is the world body.
      const std::string n1 = safeObjName(model, mjOBJ_BODY, obj1id, "body" + std::to_string(obj1id));
      const std::string n2 =
          (obj2id == 0) ? "world" : safeObjName(model, mjOBJ_BODY, obj2id, "body" + std::to_string(obj2id));
      return n1 + "_" + n2 + "_connect_constraint";
    }
    case mjEQ_WELD:
    {
      // obj1id / obj2id are body IDs; body 0 is the world body.
      const std::string n1 = safeObjName(model, mjOBJ_BODY, obj1id, "body" + std::to_string(obj1id));
      const std::string n2 =
          (obj2id == 0) ? "world" : safeObjName(model, mjOBJ_BODY, obj2id, "body" + std::to_string(obj2id));
      return n1 + "_" + n2 + "_weld_constraint";
    }
    default:
      return "equality_" + std::to_string(eq_id) + "_constraint";
  }
}

std::string ConstraintViolationsPublisherPlugin::constraintTypeString(const mjModel* model, int eq_id) const
{
  switch (model->eq_type[eq_id])
  {
    case mjEQ_JOINT:
      return "joint";
    case mjEQ_TENDON:
      return "tendon";
    case mjEQ_WELD:
      return "weld";
    case mjEQ_CONNECT:
      return "connect";
    default:
      return "unknown";
  }
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ConstraintViolationsPublisherPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
