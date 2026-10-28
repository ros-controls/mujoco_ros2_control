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

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace
{

// A plugin written before update_write() existed, overriding only the deprecated update().
class LegacyPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr, const mjModel*, mjData*) override
  {
    return true;
  }
  void update(const mjModel*, mjData*) override
  {
    ++update_calls;
  }
  void cleanup() override
  {
  }
  int update_calls{ 0 };
};

}  // namespace

TEST(PluginBaseTest, UpdateWriteForwardsToDeprecatedUpdate)
{
  LegacyPlugin plugin;
  plugin.update_write(nullptr, nullptr);
  EXPECT_EQ(plugin.update_calls, 1);
}
