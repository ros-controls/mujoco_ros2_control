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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_msgs/srv/apply_external_wrench.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mujoco_ros2_control_plugins/external_wrench_plugin.hpp"

namespace
{
// A free-floating sphere (nv = 6: 3 rotational + 3 translational DOFs).
// Body orientation is identity so body-frame and world-frame forces coincide.
constexpr const char* kMjcf = R"(
<mujoco model="external_wrench_test">
  <worldbody>
    <body name="test_body" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)";
}  // namespace

class ExternalWrenchPluginTest : public ::testing::Test
{
protected:
  using ApplyExternalWrench = mujoco_ros2_control_msgs::srv::ApplyExternalWrench;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    char error[1024] = { 0 };
    mjSpec* spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;

    model_ = mj_compile(spec, nullptr);
    if (model_ == nullptr)
    {
      const char* ce = mjs_getError(spec);
      mj_deleteSpec(spec);
      FAIL() << (ce ? ce : "mj_compile failed");
    }
    mj_deleteSpec(spec);

    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
    mj_forward(model_, data_);

    node_ = std::make_shared<rclcpp::Node>("external_wrench_test_node");
    plugin_node_ = node_->create_sub_node("external_wrench_plugin");

    // Use two executor threads so a blocking service callback (sleeping for
    // its wrench duration) does not prevent other callbacks (e.g. subscription
    // delivery) from being dispatched.
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 2);
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }
    executor_.reset();
    plugin_node_.reset();
    node_.reset();
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
  }

  /// Sends a zero-duration wrench service request (no blocking sleep in the
  /// callback) and blocks until the response arrives or the timeout elapses.
  /// Returns nullptr on timeout.
  ApplyExternalWrench::Response::SharedPtr callServiceZeroDuration(const std::string& link_name, double fx = 0.0,
                                                                   double fy = 0.0, double fz = 0.0, double tx = 0.0,
                                                                   double ty = 0.0, double tz = 0.0, double app_x = 0.0,
                                                                   double app_y = 0.0, double app_z = 0.0)
  {
    return sendRequest(link_name, fx, fy, fz, tx, ty, tz, app_x, app_y, app_z, 0, 0);
  }

  /// Sends a wrench service request with the given duration (seconds) and
  /// blocks until the response arrives (i.e. until the wrench expires) or the
  /// timeout elapses.  Returns nullptr on timeout.
  ApplyExternalWrench::Response::SharedPtr callServiceWithDuration(const std::string& link_name, double duration_sec,
                                                                   double fx = 0.0, double fy = 0.0, double fz = 0.0,
                                                                   double tx = 0.0, double ty = 0.0, double tz = 0.0)
  {
    const int32_t sec = static_cast<int32_t>(duration_sec);
    const uint32_t nanosec = static_cast<uint32_t>((duration_sec - sec) * 1e9);
    return sendRequest(link_name, fx, fy, fz, tx, ty, tz, 0.0, 0.0, 0.0, sec, nanosec);
  }

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

private:
  ApplyExternalWrench::Response::SharedPtr sendRequest(const std::string& link_name, double fx, double fy, double fz,
                                                       double tx, double ty, double tz, double app_x, double app_y,
                                                       double app_z, int32_t dur_sec, uint32_t dur_nsec)
  {
    auto client = plugin_node_->create_client<ApplyExternalWrench>("apply_wrench");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return nullptr;
    }

    auto req = std::make_shared<ApplyExternalWrench::Request>();
    req->link_name = link_name;
    req->wrench.force.x = fx;
    req->wrench.force.y = fy;
    req->wrench.force.z = fz;
    req->wrench.torque.x = tx;
    req->wrench.torque.y = ty;
    req->wrench.torque.z = tz;
    req->application_point.x = app_x;
    req->application_point.y = app_y;
    req->application_point.z = app_z;
    req->duration.sec = dur_sec;
    req->duration.nanosec = dur_nsec;

    auto future = client->async_send_request(req);

    // The background executor handles the service callback; just poll the future.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline)
    {
    }

    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
    {
      return nullptr;
    }
    return future.get();
  }

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

// ---------------------------------------------------------------------------
// init / cleanup
// ---------------------------------------------------------------------------

TEST_F(ExternalWrenchPluginTest, InitSucceeds)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

// ---------------------------------------------------------------------------
// Service validation
// ---------------------------------------------------------------------------

TEST_F(ExternalWrenchPluginTest, ServiceRejectsUnknownBodyName)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto resp = callServiceZeroDuration("nonexistent_body", /*fx=*/1.0);
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success);
  EXPECT_FALSE(resp->message.empty());

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, ServiceAcceptsValidBodyName)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto resp = callServiceZeroDuration("test_body", /*fx=*/1.0);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);
  EXPECT_FALSE(resp->message.empty());

  plugin.cleanup();
}

// ---------------------------------------------------------------------------
// qfrc_applied bookkeeping
// ---------------------------------------------------------------------------

TEST_F(ExternalWrenchPluginTest, UpdateAppliesForceToQfrcApplied)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // All DOFs must start at zero.
  for (int i = 0; i < model_->nv; ++i)
  {
    EXPECT_DOUBLE_EQ(data_->qfrc_applied[i], 0.0);
  }

  // Apply 10 N in body-frame X (≡ world-frame X at identity orientation).
  auto resp = callServiceZeroDuration("test_body", /*fx=*/10.0);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  plugin.update(model_, data_);

  // At least one generalised coordinate must carry a non-zero force.
  double total = 0.0;
  for (int i = 0; i < model_->nv; ++i)
  {
    total += std::abs(data_->qfrc_applied[i]);
  }
  EXPECT_GT(total, 0.0) << "qfrc_applied should be non-zero after applying a force";

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, UpdateUndoesPreviousContributionOnNextCall)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto resp = callServiceZeroDuration("test_body", /*fx=*/10.0);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  // First update: wrench applied, then immediately expired (zero duration).
  plugin.update(model_, data_);

  // Second update: plugin subtracts its saved contribution; no active wrenches remain.
  plugin.update(model_, data_);

  for (int i = 0; i < model_->nv; ++i)
  {
    EXPECT_DOUBLE_EQ(data_->qfrc_applied[i], 0.0) << "DOF " << i << " not zeroed after wrench expiry";
  }

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, MultipleWrenchesAccumulateLinearly)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // Queue two equal wrenches simultaneously.
  {
    auto r = callServiceZeroDuration("test_body", /*fx=*/5.0);
    ASSERT_NE(r, nullptr);
    ASSERT_TRUE(r->success);
  }
  {
    auto r = callServiceZeroDuration("test_body", /*fx=*/5.0);
    ASSERT_NE(r, nullptr);
    ASSERT_TRUE(r->success);
  }

  // First update: both wrenches active, both expire (zero duration).
  plugin.update(model_, data_);

  double total_both = 0.0;
  for (int i = 0; i < model_->nv; ++i)
  {
    total_both += std::abs(data_->qfrc_applied[i]);
  }

  // Second update clears state.
  plugin.update(model_, data_);

  // Queue a single wrench of the same magnitude.
  {
    auto r = callServiceZeroDuration("test_body", /*fx=*/5.0);
    ASSERT_NE(r, nullptr);
    ASSERT_TRUE(r->success);
  }

  plugin.update(model_, data_);

  double total_one = 0.0;
  for (int i = 0; i < model_->nv; ++i)
  {
    total_one += std::abs(data_->qfrc_applied[i]);
  }

  ASSERT_GT(total_one, 0.0) << "Single wrench should produce non-zero qfrc_applied";
  // Two equal wrenches must contribute exactly twice as much as one.
  EXPECT_NEAR(total_both, 2.0 * total_one, 1e-9);

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, TorqueOnlyWrenchAppliesRotationalForce)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // Apply only a torque (no force) about body-frame Z.
  auto resp = callServiceZeroDuration("test_body", /*fx=*/0.0, /*fy=*/0.0, /*fz=*/0.0,
                                      /*tx=*/0.0, /*ty=*/0.0, /*tz=*/5.0);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  plugin.update(model_, data_);

  double total = 0.0;
  for (int i = 0; i < model_->nv; ++i)
  {
    total += std::abs(data_->qfrc_applied[i]);
  }
  EXPECT_GT(total, 0.0) << "qfrc_applied should be non-zero after applying a torque";

  plugin.cleanup();
}

// ---------------------------------------------------------------------------
// Marker visualisation
// ---------------------------------------------------------------------------

TEST_F(ExternalWrenchPluginTest, ForceMarkerPublishedForActiveWrench)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  // Allow the publisher to be discovered by potential subscribers.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::atomic<bool> received{ false };
  MarkerArray received_msg;
  std::mutex msg_mutex;
  // The plugin publishes on "~/wrench_markers" relative to plugin_node_.
  auto sub =
      plugin_node_->create_subscription<MarkerArray>("~/wrench_markers", 10, [&](const MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msg_mutex);
        received_msg = *msg;
        received.store(true, std::memory_order_release);
      });

  // Send a wrench with a 200 ms duration from a background thread so the
  // sleeping service callback does not prevent update() from running in the
  // main thread.  The MultiThreadedExecutor has 2 worker threads, so one can
  // service the sleeping callback while the other delivers the marker message.
  std::thread svc_thread([this]() { callServiceWithDuration("test_body", 0.2, /*fx=*/10.0); });

  // Wait briefly for the service request to reach the server and the wrench
  // to be pushed into the pending queue.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Call update() while the wrench is still active (end_time is 200 ms away).
  plugin.update(model_, data_);

  // Wait for the marker message; allow the full service duration to elapse
  // so the second executor thread can deliver the subscription callback.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (!received.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  svc_thread.join();

  ASSERT_TRUE(received.load()) << "No MarkerArray received after update with an active force wrench";

  std::lock_guard<std::mutex> lock(msg_mutex);
  ASSERT_FALSE(received_msg.markers.empty());

  // Expect at least one ARROW marker in the "force" namespace.
  bool found_force_arrow = false;
  for (const auto& m : received_msg.markers)
  {
    if (m.ns == "force" && m.type == visualization_msgs::msg::Marker::ARROW &&
        m.action == visualization_msgs::msg::Marker::ADD)
    {
      found_force_arrow = true;
      EXPECT_FLOAT_EQ(m.color.r, 1.0f);
      EXPECT_FLOAT_EQ(m.color.g, 0.0f);
      EXPECT_FLOAT_EQ(m.color.b, 0.0f);
      break;
    }
  }
  EXPECT_TRUE(found_force_arrow) << "Expected a red force ARROW marker";

  plugin.cleanup();
  sub.reset();
}

TEST_F(ExternalWrenchPluginTest, DeleteAllMarkerPublishedWhenNoWrenchesActive)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::vector<MarkerArray> received;
  std::mutex received_mutex;
  auto sub =
      plugin_node_->create_subscription<MarkerArray>("~/wrench_markers", 10, [&](const MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(received_mutex);
        received.push_back(*msg);
      });

  // Update with no active wrenches: must publish a single DELETEALL marker.
  plugin.update(model_, data_);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  bool got_msg = false;
  while (!got_msg && std::chrono::steady_clock::now() < deadline)
  {
    {
      std::lock_guard<std::mutex> lock(received_mutex);
      got_msg = !received.empty();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  ASSERT_TRUE(got_msg) << "No MarkerArray received after update with no active wrenches";

  std::lock_guard<std::mutex> lock(received_mutex);
  const auto& msg = received.back();
  ASSERT_FALSE(msg.markers.empty());
  EXPECT_EQ(msg.markers[0].action, visualization_msgs::msg::Marker::DELETEALL);

  plugin.cleanup();
  sub.reset();
}
