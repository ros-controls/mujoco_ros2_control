/**
 * Copyright (c) 2026, Dylan Gallagher.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MUJOCO_ROS2_CONTROL__RENDER_LOOP_EXIT_HPP_
#define MUJOCO_ROS2_CONTROL__RENDER_LOOP_EXIT_HPP_

#include <atomic>
#include <memory>

#include <rclcpp/context.hpp>

namespace mujoco_ros2_control::detail
{

inline bool handle_render_loop_exit(std::atomic<int>& exit_request,
                                    const std::atomic<bool>& explicit_shutdown_requested,
                                    const std::shared_ptr<rclcpp::Context>& context)
{
  // MuJoCo sets exitrequest when the window closes, so it cannot distinguish a user close from
  // MujocoSimulation::shutdown(). The owner marks the latter before waking RenderLoop().
  exit_request.store(1);
  if (explicit_shutdown_requested.load())
  {
    return false;
  }

  return context && context->shutdown("MuJoCo rendering window closed");
}

}  // namespace mujoco_ros2_control::detail

#endif  // MUJOCO_ROS2_CONTROL__RENDER_LOOP_EXIT_HPP_
