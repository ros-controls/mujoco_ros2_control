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
#include <mujoco_ros2_control_msgs/msg/external_wrench.hpp>
#include <mujoco_ros2_control_msgs/msg/external_wrench_array.hpp>
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
  using ExternalWrench = mujoco_ros2_control_msgs::msg::ExternalWrench;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  static void SetUpTestCase()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestCase()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("external_wrench_test_node");
    plugin_node_ = node_->create_sub_node("external_wrench_plugin");

    // Use two executor threads so a blocking service callback (sleeping for
    // its wrench duration) does not prevent other callbacks (e.g. subscription
    // delivery) from being dispatched.
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 2);
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
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

    ExternalWrench ew;
    ew.wrench.header.frame_id = link_name;
    ew.wrench.wrench.force.x = fx;
    ew.wrench.wrench.force.y = fy;
    ew.wrench.wrench.force.z = fz;
    ew.wrench.wrench.torque.x = tx;
    ew.wrench.wrench.torque.y = ty;
    ew.wrench.wrench.torque.z = tz;
    ew.application_point.x = app_x;
    ew.application_point.y = app_y;
    ew.application_point.z = app_z;
    ew.duration.sec = dur_sec;
    ew.duration.nanosec = dur_nsec;

    auto req = std::make_shared<ApplyExternalWrench::Request>();
    req->wrenches.external_wrenches.push_back(ew);

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

TEST_F(ExternalWrenchPluginTest, InitSucceeds)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

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

TEST_F(ExternalWrenchPluginTest, UpdateAppliesForceToXfrcApplied)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // All xfrc_applied entries must start at zero.
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    EXPECT_DOUBLE_EQ(data_->xfrc_applied[i], 0.0);
  }

  // Apply 10 N in body-frame X (≡ world-frame X at identity orientation).
  auto resp = callServiceZeroDuration("test_body", /*fx=*/10.0);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  plugin.update(model_, data_);

  // At least one xfrc_applied entry must carry a non-zero value.
  double total = 0.0;
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    total += std::abs(data_->xfrc_applied[i]);
  }
  EXPECT_GT(total, 0.0) << "xfrc_applied should be non-zero after applying a force";

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

  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    EXPECT_DOUBLE_EQ(data_->xfrc_applied[i], 0.0) << "xfrc_applied[" << i << "] not zeroed after wrench expiry";
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
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    total_both += std::abs(data_->xfrc_applied[i]);
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
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    total_one += std::abs(data_->xfrc_applied[i]);
  }

  ASSERT_GT(total_one, 0.0) << "Single wrench should produce non-zero xfrc_applied";
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
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    total += std::abs(data_->xfrc_applied[i]);
  }
  EXPECT_GT(total, 0.0) << "xfrc_applied should be non-zero after applying a torque";

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, SingleCallWithMultipleWrenchesAppliesAll)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // Send two wrenches in a single service call: 5 N in X and 5 N in Y.
  auto client = plugin_node_->create_client<ApplyExternalWrench>("apply_wrench");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  ExternalWrench ew1;
  ew1.wrench.header.frame_id = "test_body";
  ew1.wrench.wrench.force.x = 5.0;

  ExternalWrench ew2;
  ew2.wrench.header.frame_id = "test_body";
  ew2.wrench.wrench.force.y = 5.0;

  auto req = std::make_shared<ApplyExternalWrench::Request>();
  req->wrenches.external_wrenches = { ew1, ew2 };

  auto future = client->async_send_request(req);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready &&
         std::chrono::steady_clock::now() < deadline)
  {
  }
  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  plugin.update(model_, data_);

  double total = 0.0;
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    total += std::abs(data_->xfrc_applied[i]);
  }
  EXPECT_GT(total, 0.0) << "Both wrenches in the batch should produce non-zero xfrc_applied";

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, SingleCallRejectsIfAnyBodyNameInvalid)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto client = plugin_node_->create_client<ApplyExternalWrench>("apply_wrench");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  // First wrench is valid, second has an unknown body name.
  ExternalWrench ew1;
  ew1.wrench.header.frame_id = "test_body";
  ew1.wrench.wrench.force.x = 5.0;

  ExternalWrench ew2;
  ew2.wrench.header.frame_id = "nonexistent_body";
  ew2.wrench.wrench.force.x = 5.0;

  auto req = std::make_shared<ApplyExternalWrench::Request>();
  req->wrenches.external_wrenches = { ew1, ew2 };

  auto future = client->async_send_request(req);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready &&
         std::chrono::steady_clock::now() < deadline)
  {
  }
  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  auto resp = future.get();
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success) << "Request with one invalid body name must be rejected entirely";
  EXPECT_FALSE(resp->message.empty());

  // No wrench should have been applied — xfrc_applied must remain zero.
  plugin.update(model_, data_);
  for (int i = 0; i < model_->nbody * 6; ++i)
  {
    EXPECT_DOUBLE_EQ(data_->xfrc_applied[i], 0.0) << "xfrc_applied[" << i << "] must be zero after rejected batch";
  }

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, PublishMarkersForActiveForceWrench)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // Send a wrench with a 200 ms duration from a background thread so the
  // sleeping service callback does not block update() in the main thread.
  std::thread svc_thread([this]() { callServiceWithDuration("test_body", 0.2, /*fx=*/10.0); });

  // Wait briefly for the wrench to reach the pending queue.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Run update() while the wrench is still active.
  plugin.update(model_, data_);

  // Collect markers via the new aggregation API.
  MarkerArray markers;
  plugin.publish_markers(markers);

  svc_thread.join();

  ASSERT_FALSE(markers.markers.empty()) << "Expected markers for an active force wrench";

  // Expect at least one ARROW marker in the "external_wrench/force" namespace.
  bool found_force_arrow = false;
  for (const auto& m : markers.markers)
  {
    if (m.ns == "external_wrench/force" && m.type == visualization_msgs::msg::Marker::ARROW &&
        m.action == visualization_msgs::msg::Marker::ADD)
    {
      found_force_arrow = true;
      EXPECT_FLOAT_EQ(m.color.r, 1.0f);
      EXPECT_FLOAT_EQ(m.color.g, 0.0f);
      EXPECT_FLOAT_EQ(m.color.b, 0.0f);
      break;
    }
  }
  EXPECT_TRUE(found_force_arrow) << "Expected a red force ARROW marker in the 'external_wrench/force' namespace";

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, PublishMarkersContributesNothingWhenNoWrenchesActive)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  // No service calls — no active wrenches.
  plugin.update(model_, data_);

  // publish_markers() must not append anything when there are no active wrenches.
  // Marker lifetime-based expiry (managed by the system interface) handles cleanup
  // in RViz; the plugin itself does not emit a DELETEALL.
  MarkerArray markers;
  plugin.publish_markers(markers);

  EXPECT_TRUE(markers.markers.empty()) << "Expected no markers when there are no active wrenches";

  plugin.cleanup();
}

TEST_F(ExternalWrenchPluginTest, PublishMarkersAppendsToExistingArray)
{
  mujoco_ros2_control_plugins::ExternalWrenchPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  std::thread svc_thread([this]() { callServiceWithDuration("test_body", 0.2, /*fx=*/5.0, /*fy=*/0.0, /*fz=*/0.0); });
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  plugin.update(model_, data_);

  // Pre-populate the array with a sentinel marker (simulates another plugin's contribution).
  MarkerArray markers;
  visualization_msgs::msg::Marker sentinel;
  sentinel.ns = "other_plugin";
  sentinel.id = 99;
  markers.markers.push_back(sentinel);

  plugin.publish_markers(markers);
  svc_thread.join();

  // The sentinel must still be present, and the plugin must have appended its own markers.
  ASSERT_GT(markers.markers.size(), 1u) << "Plugin must append to, not replace, the existing array";
  EXPECT_EQ(markers.markers.front().ns, "other_plugin") << "Sentinel marker must be preserved";
}
