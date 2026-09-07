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

#include "fts_grav_comp_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool FtsGravCompPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  const std::string param_prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
  const std::string fts_name_param = param_prefix + "fts_name";
  if (!node_->has_parameter(fts_name_param))
  {
    RCLCPP_ERROR(logger_, "FtsGravCompPlugin failed to initialize. Param `fts_name`was not defined in the params file");
    return false;
  }
  std::string fts_name = node_->get_parameter(fts_name_param).as_string();

  std::string sensor_name_force = fts_name + "_force";
  std::string sensor_name_torque = fts_name + "_torque";

  // get sensor ids and make sure they are valid
  int sensor_id_force_ = mj_name2id(model, mjOBJ_SENSOR, sensor_name_force.c_str());
  if (sensor_id_force_ == -1)
  {
    RCLCPP_ERROR(logger_, "FtsGravCompPlugin failed to initialize. Force sensor name %s not found in the model.",
                 sensor_name_force.c_str());
    return false;
  }
  int sensor_id_torque_ = mj_name2id(model, mjOBJ_SENSOR, sensor_name_torque.c_str());
  if (sensor_id_torque_ == -1)
  {
    RCLCPP_ERROR(logger_, "FtsGravCompPlugin failed to initialize. Torque sensor name %s not found in the model.",
                 sensor_name_torque.c_str());
    return false;
  }

  // get sensor addresses
  sensor_adr_force_ = model->sensor_adr[sensor_id_force_];
  sensor_adr_torque_ = model->sensor_adr[sensor_id_torque_];
  sensor_dim_force_ = model->sensor_dim[sensor_id_force_];
  sensor_dim_torque_ = model->sensor_dim[sensor_id_torque_];

  fts_wrench_publisher_ = node_->create_publisher<geometry_msgs::msg::Wrench>("fts_wrench", 10);

  // Initialize the last publish time
  last_publish_time_ = node_->get_clock()->now();

  RCLCPP_INFO(logger_, "FtsGravCompPlugin initialized. Publishing to topic 'fts_wrench' every %.3f second(s).",
              publish_period_.seconds());

  return true;
}

void FtsGravCompPlugin::update(const mjModel* /*model*/, mjData* data)
{
  auto current_time = node_->get_clock()->now();
  auto elapsed = current_time - last_publish_time_;

  // Check if it's time to publish
  if (elapsed >= publish_period_)
  {
    const mjtNum* sensordata_force = data->sensordata + sensor_adr_force_;
    const mjtNum* sensordata_torque = data->sensordata + sensor_adr_torque_;
    // everything must be negated to handle the way force and torque sensors are set up
    wrench_.force.x = -sensordata_force[0];
    wrench_.force.y = -sensordata_force[1];
    wrench_.force.z = -sensordata_force[2];
    wrench_.torque.x = -sensordata_torque[0];
    wrench_.torque.y = -sensordata_torque[1];
    wrench_.torque.z = -sensordata_torque[2];

    fts_wrench_publisher_->publish(wrench_);

    last_publish_time_ = current_time;
  }
}

void FtsGravCompPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "FtsGravCompPlugin cleanup. Published %lu messages total.", message_count_);

  fts_wrench_publisher_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::FtsGravCompPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
