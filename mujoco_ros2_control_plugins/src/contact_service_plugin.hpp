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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__CONTACT_SERVICE_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__CONTACT_SERVICE_PLUGIN_HPP_

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <mujoco_ros2_control_msgs/srv/get_contacts.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace mujoco_ros2_control_plugins
{

/**
 * @brief Plugin that retrieves MuJoCo contact information.
 *
 * Exposes the ROS 2 service ~/get_contacts of type
 * mujoco_ros2_control_msgs/srv/GetContacts.
 *
 * The request is empty; each call asks for a fresh simulation snapshot.
 *
 * Service response
 * ----------------
 *   success  - false when the simulation does not process the request before
 *              the timeout.
 *   message  - empty on success; describes a failed capture otherwise.
 *   snapshot - contact data in mjData.contact order, expressed in the MuJoCo
 *              world/contact frames described by the Contact message.
 *
 * Each request asks the simulation thread to capture the current mjData contact
 * array. The callback waits for the corresponding capture generation, then
 * returns the cached snapshot without accessing physics-owned mjData.
 *
 * A request fails if the simulation thread does not process the capture within
 * the one-second timeout. Concurrent requests may observe the same capture
 * generation when they are pending at the same time.
 *
 * Implementation notes
 * --------------------
 * update() runs in the simulation thread and returns immediately when no
 * capture is pending. Contact data is copied while holding the cache mutex; the
 * service callback copies that cache before constructing the ROS response.
 */
class ContactServicePlugin : public MuJoCoROS2ControlPluginBase
{
public:
  ContactServicePlugin() = default;
  ~ContactServicePlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  using GetContacts = mujoco_ros2_control_msgs::srv::GetContacts;

  /**
   * @brief Lifetime gate shared with service callbacks.
   *
   * The service callback captures this gate by shared ownership. Once cleanup
   * closes the gate, callbacks that have not yet started can reject the request
   * without dereferencing the plugin, while cleanup waits for callbacks that
   * already acquired a lease.
   */
  struct CallbackGate
  {
    bool tryAcquire();
    void release();
    void close();
    bool isClosing() const;
    void waitForIdle();

    std::atomic_bool closing{ false };
    std::mutex mutex;
    std::condition_variable cv;
    std::size_t active_callbacks{ 0 };
  };

  /// RAII lease ensuring cleanup waits for an active service callback.
  class CallbackLease
  {
  public:
    explicit CallbackLease(std::shared_ptr<CallbackGate> gate);
    ~CallbackLease();

    CallbackLease(const CallbackLease&) = delete;
    CallbackLease& operator=(const CallbackLease&) = delete;

    explicit operator bool() const;

  private:
    std::shared_ptr<CallbackGate> gate_;
    bool acquired_{ false };
  };

  /// Cached contact data copied from mjData during update().
  struct CachedContact
  {
    mjContact contact;
  };

  /// Service callback; runs in a ROS executor thread.
  void handleGetContacts(const GetContacts::Request::SharedPtr request, GetContacts::Response::SharedPtr response);

  // ROS interfaces
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("ContactServicePlugin") };
  rclcpp::Service<GetContacts>::SharedPtr service_;
  std::shared_ptr<CallbackGate> callback_gate_;

  // Model pointer (const, valid for simulation lifetime)
  const mjModel* model_{ nullptr };

  // Contact cache shared by update() and the service callback.
  std::mutex cache_mutex_;
  std::condition_variable capture_cv_;
  std::vector<CachedContact> cached_contacts_;
  // Completed snapshot sequence used to match requests with captures.
  uint64_t capture_generation_{ 0 };

  // Fast-path flag set by the service callback and consumed by update().
  std::atomic_bool service_requested_{ false };
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__CONTACT_SERVICE_PLUGIN_HPP_
