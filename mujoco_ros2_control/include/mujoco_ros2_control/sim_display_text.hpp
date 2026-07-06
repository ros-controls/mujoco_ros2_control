/**
 * Copyright (c) 2026 PAL Robotics S.L.
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

#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>

namespace mujoco_ros2_control
{

/**
 * @brief Composes the title/content text pair for the native-viewer status overlay.
 *
 * Extracted as a pure, dependency-free function so the overlay layout and number
 * formatting can be unit tested without a live OpenGL/GLFW rendering context (the
 * overlay itself is only drawn in non-headless mode). MuJoCo's overlay API renders
 * the two strings side by side: each newline-separated row in @p title is paired
 * with the row at the same position in the returned content string, so the two must
 * always contain the same number of rows.
 *
 * @param running     Whether the physics loop is currently advancing. Drives the
 *                    "Status" row ("Running"/"Paused").
 * @param step_count  Number of physics steps taken so far (mjData step counter).
 * @param sim_time    Current simulation clock in seconds (mjData::time).
 * @param desired_pct Target real-time percentage — either the UI slider value or the
 *                    `sim_speed_factor` override, resolved by the caller.
 * @param actual_pct  Measured real-time percentage (100 / measured_slowdown). The
 *                    caller passes 0 while paused.
 * @param ncon        Number of active contacts in the current step (mjData::ncon).
 * @return A pair of {title, content} strings ready to hand to MuJoCo's user-text overlay.
 */
inline std::pair<std::string, std::string> compose_sim_display_text(bool running, uint64_t step_count, double sim_time,
                                                                    double desired_pct, double actual_pct, int ncon)
{
  char sim_time_buf[24], desired_buf[16], actual_buf[16];
  std::snprintf(sim_time_buf, sizeof(sim_time_buf), "%.3f s", sim_time);
  std::snprintf(desired_buf, sizeof(desired_buf), "%.1f%%", desired_pct);
  std::snprintf(actual_buf, sizeof(actual_buf), "%.1f%%", actual_pct);

  const std::string title = "Status\nSteps\nSim Time\nDesired Speed\nActual Speed\nContacts";
  const std::string content = std::string(running ? "Running" : "Paused") + "\n" + std::to_string(step_count) + "\n" +
                              sim_time_buf + "\n" + desired_buf + "\n" + actual_buf + "\n" + std::to_string(ncon);
  return { title, content };
}

}  // namespace mujoco_ros2_control
