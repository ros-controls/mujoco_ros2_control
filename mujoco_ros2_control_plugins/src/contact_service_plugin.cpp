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

#include "contact_service_plugin.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include <rmw/types.h>
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool ContactServicePlugin::CallbackGate::tryAcquire()
{
  if (closing.load(std::memory_order_acquire))
  {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex);
  if (closing.load(std::memory_order_acquire))
  {
    return false;
  }

  ++active_callbacks;
  return true;
}

void ContactServicePlugin::CallbackGate::release()
{
  bool idle = false;
  {
    std::lock_guard<std::mutex> lock(mutex);
    --active_callbacks;
    idle = active_callbacks == 0;
  }

  if (idle)
  {
    cv.notify_all();
  }
}

void ContactServicePlugin::CallbackGate::close()
{
  closing.store(true, std::memory_order_release);
}

bool ContactServicePlugin::CallbackGate::isClosing() const
{
  return closing.load(std::memory_order_acquire);
}

void ContactServicePlugin::CallbackGate::waitForIdle()
{
  std::unique_lock<std::mutex> lock(mutex);
  cv.wait(lock, [this]() { return active_callbacks == 0; });
}

ContactServicePlugin::CallbackLease::CallbackLease(std::shared_ptr<CallbackGate> gate) : gate_(std::move(gate))
{
  acquired_ = gate_->tryAcquire();
}

ContactServicePlugin::CallbackLease::~CallbackLease()
{
  if (acquired_)
  {
    gate_->release();
  }
}

ContactServicePlugin::CallbackLease::operator bool() const
{
  return acquired_;
}

bool ContactServicePlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  model_ = model;
  callback_gate_ = std::make_shared<CallbackGate>();

  if (model_->nconmax > 0)
  {
    cached_contacts_.reserve(static_cast<std::size_t>(model_->nconmax));
  }

  const auto callback_gate = callback_gate_;
  service_ = node_->create_service<GetContacts>(
      "get_contacts",
      [this, callback_gate](const std::shared_ptr<rmw_request_id_t> /*request_header*/,
                            const GetContacts::Request::SharedPtr request, GetContacts::Response::SharedPtr response) {
        CallbackLease lease(callback_gate);
        if (!lease)
        {
          response->success = false;
          response->message = "Contact service is shutting down.";
          return;
        }
        handleGetContacts(request, response);
      });

  RCLCPP_INFO(node_->get_logger(), "ContactServicePlugin initialised. Service available at '%s'.",
              service_->get_service_name());
  return true;
}

void ContactServicePlugin::update(const mjModel* /*model_arg*/, mjData* data)
{
  // Step 0 - skip the cache update when no service request is pending.
  if (!service_requested_.load(std::memory_order_acquire))
  {
    return;
  }

  // Step 1 - acquire the cache mutex without blocking the simulation thread.
  //          The scoped lock keeps the cache and generation update atomic.
  {
    std::unique_lock<std::mutex> lock(cache_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      return;
    }

    // A previous update may have consumed the request while this update waited
    // for the cache mutex.
    if (!service_requested_.exchange(false, std::memory_order_acq_rel))
    {
      return;
    }

    // Step 2 - capture one consistent snapshot of the mjData contact array.
    cached_contacts_.resize(static_cast<std::size_t>(data->ncon));

    for (int i = 0; i < data->ncon; ++i)
    {
      auto& cached = cached_contacts_[static_cast<std::size_t>(i)];
      cached.contact = data->contact[i];
    }

    // Mark the snapshot complete only after the cache is fully written.
    ++capture_generation_;
  }

  // Step 3 - notify waiting service callbacks after releasing the mutex.
  capture_cv_.notify_all();
}

// ---------------------------------------------------------------------------
// Service callback
// ---------------------------------------------------------------------------

void ContactServicePlugin::handleGetContacts(const GetContacts::Request::SharedPtr /*request*/,
                                             GetContacts::Response::SharedPtr response)
{
  // Register the request while holding the cache mutex so the requested
  // generation and capture flag form one synchronized operation.
  std::vector<CachedContact> cached_contacts;
  {
    std::unique_lock<std::mutex> lock(cache_mutex_);
    const uint64_t requested_generation = capture_generation_ + 1;
    service_requested_.store(true, std::memory_order_release);

    // wait_for() releases cache_mutex_ while update() performs the capture and
    // reacquires it before evaluating the predicate.
    constexpr auto capture_timeout = std::chrono::seconds(1);
    const bool captured = capture_cv_.wait_for(lock, capture_timeout, [this, requested_generation]() {
      return capture_generation_ >= requested_generation || callback_gate_->isClosing();
    });

    if (callback_gate_->isClosing())
    {
      response->success = false;
      response->message = "Contact service is shutting down.";
      response->snapshot = mujoco_ros2_control_msgs::msg::ContactArray();
      return;
    }

    if (!captured)
    {
      response->success = false;
      response->message = "Timed out waiting for a fresh contact snapshot.";
      response->snapshot = mujoco_ros2_control_msgs::msg::ContactArray();
      return;
    }

    // Copy the cache while protected, then build the ROS response after the
    // lock is released so serialization/conversion does not delay update().
    cached_contacts = cached_contacts_;
  }

  response->success = true;
  response->message.clear();
  response->snapshot.contacts.resize(cached_contacts.size());

  for (std::size_t i = 0; i < cached_contacts.size(); ++i)
  {
    const auto& cached = cached_contacts[i];
    const auto& contact = cached.contact;
    auto& output = response->snapshot.contacts[i];

    output.id = static_cast<int32_t>(i);
    for (std::size_t side = 0; side < 2; ++side)
    {
      output.geom_ids[side] = contact.geom[side];
      output.flex_ids[side] = contact.flex[side];
      output.elem_ids[side] = contact.elem[side];
      output.vert_ids[side] = contact.vert[side];
      output.body_ids[side] = -1;

      if (contact.geom[side] >= 0)
      {
        const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, contact.geom[side]);
        output.geom_names[side] = geom_name ? geom_name : "";

        output.body_ids[side] = model_->geom_bodyid[contact.geom[side]];
        const char* body_name = mj_id2name(model_, mjOBJ_BODY, output.body_ids[side]);
        output.body_names[side] = body_name ? body_name : "";
      }
    }

    output.distance = contact.dist;
    output.position.x = contact.pos[0];
    output.position.y = contact.pos[1];
    output.position.z = contact.pos[2];

    output.normal.x = contact.frame[0];
    output.normal.y = contact.frame[1];
    output.normal.z = contact.frame[2];
    output.tangent_1.x = contact.frame[3];
    output.tangent_1.y = contact.frame[4];
    output.tangent_1.z = contact.frame[5];
    output.tangent_2.x = contact.frame[6];
    output.tangent_2.y = contact.frame[7];
    output.tangent_2.z = contact.frame[8];

    output.dimension = contact.dim;
    output.exclude = contact.exclude;
    output.include_margin = contact.includemargin;
    std::copy(std::begin(contact.friction), std::end(contact.friction), output.friction.begin());
    output.solref.assign(std::begin(contact.solref), std::end(contact.solref));
    output.solref_friction.assign(std::begin(contact.solreffriction), std::end(contact.solreffriction));
    output.solimp.assign(std::begin(contact.solimp), std::end(contact.solimp));
  }
}

// ---------------------------------------------------------------------------
// Cleanup
// ---------------------------------------------------------------------------

void ContactServicePlugin::cleanup()
{
  RCLCPP_INFO(logger_, "ContactServicePlugin cleanup.");

  const auto callback_gate = callback_gate_;
  if (callback_gate)
  {
    callback_gate->close();
  }

  service_.reset();
  service_requested_.store(false, std::memory_order_release);
  capture_cv_.notify_all();

  if (callback_gate)
  {
    callback_gate->waitForIdle();
  }

  node_.reset();
  model_ = nullptr;
  callback_gate_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::ContactServicePlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
