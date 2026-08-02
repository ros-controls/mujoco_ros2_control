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

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include <mujoco_ros2_control_msgs/srv/get_contacts.hpp>
#include <rclcpp/rclcpp.hpp>

#include "contact_service_plugin.hpp"

namespace
{
constexpr const char* kMjcf = R"(
<mujoco model="contact_service_test">
  <worldbody>
    <geom name="ground" type="plane" size="2 2 0.1"/>
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom name="ball_geom" type="sphere" size="0.1" mass="1"/>
    </body>
  </worldbody>
  <contact>
    <pair geom1="ground" geom2="ball_geom" condim="6"
          friction="0.8 0.6 0.04 0.03 0.02"
          solref="0.03 0.9" solreffriction="0.04 1.1"
          solimp="0.8 0.95 0.002 0.6 3"/>
  </contact>
</mujoco>
)";
}  // namespace

class ContactServicePluginTest : public ::testing::Test
{
protected:
  using GetContacts = mujoco_ros2_control_msgs::srv::GetContacts;

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
    node_ = std::make_shared<rclcpp::Node>("contact_service_test_node");
    plugin_node_ = node_->create_sub_node("contact_service_plugin");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    char error[1024] = { 0 };
    mjSpec* spec = mj_parseXMLString(kMjcf, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;

    model_ = mj_compile(spec, nullptr);
    if (model_ == nullptr)
    {
      const char* compile_error = mjs_getError(spec);
      mj_deleteSpec(spec);
      FAIL() << (compile_error ? compile_error : "mj_compile failed");
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

  GetContacts::Response::SharedPtr call_service(mujoco_ros2_control_plugins::ContactServicePlugin& plugin,
                                                bool process_updates = true)
  {
    auto client = plugin_node_->create_client<GetContacts>("get_contacts");
    if (!client->wait_for_service(std::chrono::seconds(2)))
    {
      return nullptr;
    }

    auto future = client->async_send_request(std::make_shared<GetContacts::Request>());
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (future.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline)
    {
      if (process_updates)
      {
        plugin.update(model_, data_);
      }
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready ? future.get() : nullptr;
  }

  void set_ball_height(double height)
  {
    for (int i = 0; i < model_->njnt; ++i)
    {
      if (model_->jnt_type[i] == mjJNT_FREE)
      {
        data_->qpos[model_->jnt_qposadr[i] + 2] = height;
        mj_forward(model_, data_);
        return;
      }
    }
    FAIL() << "No free joint found";
  }

  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;

private:
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

TEST_F(ContactServicePluginTest, ReturnsEmptyInitialSnapshot)
{
  ASSERT_EQ(data_->ncon, 0);

  mujoco_ros2_control_plugins::ContactServicePlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto response = call_service(plugin);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->snapshot.contacts.empty());

  plugin.cleanup();
}

TEST_F(ContactServicePluginTest, FailsWhenCaptureIsNotProcessed)
{
  mujoco_ros2_control_plugins::ContactServicePlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  auto response = call_service(plugin, false);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_FALSE(response->message.empty());
  EXPECT_TRUE(response->snapshot.contacts.empty());

  plugin.cleanup();
}

TEST_F(ContactServicePluginTest, CleanupUnblocksInFlightRequest)
{
  auto plugin = std::make_unique<mujoco_ros2_control_plugins::ContactServicePlugin>();
  ASSERT_TRUE(plugin->init(plugin_node_, model_, data_));

  auto client = plugin_node_->create_client<GetContacts>("get_contacts");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(2)));

  auto future = client->async_send_request(std::make_shared<GetContacts::Request>());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  const auto cleanup_start = std::chrono::steady_clock::now();
  plugin->cleanup();
  const auto cleanup_duration = std::chrono::steady_clock::now() - cleanup_start;
  EXPECT_LT(cleanup_duration, std::chrono::milliseconds(500));

  plugin.reset();

  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(500)), std::future_status::ready);
  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "Contact service is shutting down.");
  EXPECT_TRUE(response->snapshot.contacts.empty());
}

TEST_F(ContactServicePluginTest, ReturnsAllContactFields)
{
  set_ball_height(0.05);
  ASSERT_GT(data_->ncon, 0);

  mujoco_ros2_control_plugins::ContactServicePlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  auto response = call_service(plugin);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  ASSERT_EQ(response->snapshot.contacts.size(), static_cast<std::size_t>(data_->ncon));

  for (int i = 0; i < data_->ncon; ++i)
  {
    const auto& expected = data_->contact[i];
    const auto& actual = response->snapshot.contacts[static_cast<std::size_t>(i)];

    EXPECT_EQ(actual.id, i);
    EXPECT_DOUBLE_EQ(actual.distance, expected.dist);
    EXPECT_DOUBLE_EQ(actual.position.x, expected.pos[0]);
    EXPECT_DOUBLE_EQ(actual.position.y, expected.pos[1]);
    EXPECT_DOUBLE_EQ(actual.position.z, expected.pos[2]);
    EXPECT_DOUBLE_EQ(actual.normal.x, expected.frame[0]);
    EXPECT_DOUBLE_EQ(actual.normal.y, expected.frame[1]);
    EXPECT_DOUBLE_EQ(actual.normal.z, expected.frame[2]);
    EXPECT_DOUBLE_EQ(actual.tangent_1.x, expected.frame[3]);
    EXPECT_DOUBLE_EQ(actual.tangent_1.y, expected.frame[4]);
    EXPECT_DOUBLE_EQ(actual.tangent_1.z, expected.frame[5]);
    EXPECT_DOUBLE_EQ(actual.tangent_2.x, expected.frame[6]);
    EXPECT_DOUBLE_EQ(actual.tangent_2.y, expected.frame[7]);
    EXPECT_DOUBLE_EQ(actual.tangent_2.z, expected.frame[8]);
    EXPECT_EQ(actual.dimension, expected.dim);
    EXPECT_EQ(actual.exclude, expected.exclude);
    EXPECT_DOUBLE_EQ(actual.include_margin, expected.includemargin);

    for (std::size_t side = 0; side < 2; ++side)
    {
      EXPECT_EQ(actual.geom_ids[side], expected.geom[side]);
      EXPECT_EQ(actual.flex_ids[side], expected.flex[side]);
      EXPECT_EQ(actual.elem_ids[side], expected.elem[side]);
      EXPECT_EQ(actual.vert_ids[side], expected.vert[side]);
      const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, expected.geom[side]);
      EXPECT_EQ(actual.geom_names[side], geom_name ? geom_name : "");
      const int body_id = model_->geom_bodyid[expected.geom[side]];
      EXPECT_EQ(actual.body_ids[side], body_id);
      const char* body_name = mj_id2name(model_, mjOBJ_BODY, body_id);
      EXPECT_EQ(actual.body_names[side], body_name ? body_name : "");
    }
    for (std::size_t j = 0; j < actual.friction.size(); ++j)
    {
      EXPECT_DOUBLE_EQ(actual.friction[j], expected.friction[j]);
    }
    ASSERT_EQ(actual.solref.size(), static_cast<std::size_t>(mjNREF));
    ASSERT_EQ(actual.solref_friction.size(), static_cast<std::size_t>(mjNREF));
    for (std::size_t j = 0; j < actual.solref.size(); ++j)
    {
      EXPECT_DOUBLE_EQ(actual.solref[j], expected.solref[j]);
      EXPECT_DOUBLE_EQ(actual.solref_friction[j], expected.solreffriction[j]);
    }
    ASSERT_EQ(actual.solimp.size(), static_cast<std::size_t>(mjNIMP));
    for (std::size_t j = 0; j < actual.solimp.size(); ++j)
    {
      EXPECT_DOUBLE_EQ(actual.solimp[j], expected.solimp[j]);
    }
  }

  plugin.cleanup();
}

TEST_F(ContactServicePluginTest, RefreshRemovesStaleContacts)
{
  mujoco_ros2_control_plugins::ContactServicePlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));

  set_ball_height(0.05);
  ASSERT_GT(data_->ncon, 0);
  auto contact_response = call_service(plugin);
  ASSERT_NE(contact_response, nullptr);
  EXPECT_TRUE(contact_response->success);
  EXPECT_FALSE(contact_response->snapshot.contacts.empty());

  set_ball_height(1.0);
  ASSERT_EQ(data_->ncon, 0);
  auto empty_response = call_service(plugin);
  ASSERT_NE(empty_response, nullptr);
  EXPECT_TRUE(empty_response->success);
  EXPECT_TRUE(empty_response->snapshot.contacts.empty());

  plugin.cleanup();
}
