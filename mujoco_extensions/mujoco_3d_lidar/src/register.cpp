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

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>
#include "mujoco_3d_lidar/3dlidar.h"

namespace mujoco::plugin::lidar
{

// Refer to
// https://github.com/google-deepmind/mujoco/blob/main/doc/changelog.rst#version-370-april-14-2026
#if mjVERSION_HEADER >= 3007000
mjPLUGIN_LIB_INIT(lidar)
#else
mjPLUGIN_LIB_INIT
#endif
{
  Lidar::RegisterPlugin();
}

}  // namespace mujoco::plugin::lidar
