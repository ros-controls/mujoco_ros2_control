/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <mutex>

#include "camera_plugin.hpp"

namespace mujoco_ros2_control_plugins
{

// Lets tests drive CameraPlugin's render-in-flight guard directly, the same way the real
// rendering thread would flip it, instead of having to make an actual render pass outlast a
// publish interval to exercise it.
class CameraPluginTestHelper
{
public:
  static void set_render_in_flight(CameraPlugin& plugin, bool in_flight)
  {
    std::lock_guard<std::mutex> lock(plugin.data_mutex_);
    plugin.render_in_flight_ = in_flight;
  }
};

}  // namespace mujoco_ros2_control_plugins
