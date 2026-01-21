/**
 * Copyright (c) 2025, United States Government, as represented by the
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

#include <hardware_interface/hardware_info.hpp>

namespace mujoco_ros2_control
{

/**
 * @brief Returns the sensor's component info for the provided sensor name, if it exists.
 */
inline std::optional<hardware_interface::ComponentInfo>
get_sensor_from_info(const hardware_interface::HardwareInfo& hardware_info, const std::string& name)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    const auto& sensor = hardware_info.sensors.at(sensor_index);
    if (hardware_info.sensors.at(sensor_index).name == name)
    {
      return sensor;
    }
  }
  return std::nullopt;
}

}  // namespace mujoco_ros2_control
