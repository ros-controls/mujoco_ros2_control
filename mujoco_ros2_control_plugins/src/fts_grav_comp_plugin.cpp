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

std::vector<std::string> FtsGravCompPlugin::get_sensor_names_from_parameters()
{
  const std::string param_namespace = "mujoco_plugins." + node_->get_sub_namespace();
  // List all parameters under mujoco_plugins.<plugin_name> recursively. A finite depth is not
  // used here because rclcpp's depth counting is off-by-one on Humble (fixed in Jazzy), so a
  // fixed depth value does not behave consistently across distros.
  auto param_names =
      node_->list_parameters({ param_namespace }, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE).names;

  // Use a set to store unique sensor names
  std::set<std::string> sensor_names_set;

  const std::string prefix = param_namespace + ".";

  for (const auto& name : param_names)
  {
    // Remove the prefix
    if (name.find(prefix) == 0)
    {
      std::string relative_path = name.substr(prefix.length());

      // Extract the first part (sensor name) before the first dot
      size_t dot_pos = relative_path.find('.');
      if (dot_pos != std::string::npos)
      {
        std::string sensor_name = relative_path.substr(0, dot_pos);

        // Skip "type" parameter
        if (sensor_name != "type")
        {
          sensor_names_set.insert(sensor_name);
        }
      }
    }
  }

  // Convert set to vector
  std::vector<std::string> sensor_names(sensor_names_set.begin(), sensor_names_set.end());

  return sensor_names;
}

bool FtsGravCompPlugin::register_fts(const mjModel* model)
{
  const auto sensor_names = get_sensor_names_from_parameters();

  const std::string param_prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
  const std::string force_suffix = "_force";
  const std::string torque_suffix = "_torque";

  fts_.resize(0);
  for (const auto& sensor_name : sensor_names)
  {
    const std::string sensor_param_prefix = param_prefix + sensor_name + ".";
    const std::string sensor_name_force = sensor_name + force_suffix;
    const std::string sensor_name_torque = sensor_name + torque_suffix;

    // get sensor ids and make sure they are valid
    int sensor_id_force = mj_name2id(model, mjOBJ_SENSOR, sensor_name_force.c_str());
    if (sensor_id_force == -1)
    {
      RCLCPP_ERROR(logger_, "Force sensor name %s not found in the model.", sensor_name_force.c_str());
      return false;
    }
    int sensor_id_torque = mj_name2id(model, mjOBJ_SENSOR, sensor_name_torque.c_str());
    if (sensor_id_torque == -1)
    {
      RCLCPP_ERROR(logger_, "Torque sensor name %s not found in the model.", sensor_name_torque.c_str());
      return false;
    }

    // grab the rest of the parameters that we need
    const std::vector<double> cog_pos_param = node_->get_parameter(sensor_param_prefix + "CoG.pos").as_double_array();
    const double cog_mass_param = node_->get_parameter(sensor_param_prefix + "CoG.mass").as_double();
    const std::string frame_id_param = node_->get_parameter(sensor_param_prefix + "frame_id").as_string();

    // make sure that the force and torque sensors are of the right dimension
    const int sensor_dim_force = model->sensor_dim[sensor_id_force];
    const int sensor_dim_torque = model->sensor_dim[sensor_id_torque];

    if ((sensor_dim_force != 3) || (sensor_dim_torque != 3))
    {
      RCLCPP_ERROR(logger_,
                   "Force and Torque sensors must be size 3. Sensors '%s' and '%s' are size %d and %d respectively.",
                   sensor_name_force.c_str(), sensor_name_torque.c_str(), sensor_dim_force, sensor_dim_torque);
      return false;
    }

    int cog_site_id = mj_name2id(model, mjOBJ_SITE, frame_id_param.c_str());
    if (cog_site_id == -1)
    {
      RCLCPP_ERROR(logger_, "Param cog.frame_id '%s' does not exist as a site in the mjcf", frame_id_param.c_str());
      return false;
    }

    FtsData fts_data;
    fts_data.sensor_name = sensor_name;
    fts_data.sensor_adr_force = model->sensor_adr[sensor_id_force];
    fts_data.sensor_adr_torque = model->sensor_adr[sensor_id_torque];
    fts_data.cog_site_id = cog_site_id;
    fts_data.cog_mass = cog_mass_param;
    fts_data.cog_pos[0] = cog_pos_param[0];
    fts_data.cog_pos[1] = cog_pos_param[1];
    fts_data.cog_pos[2] = cog_pos_param[2];
    fts_data.fts_site_id = model->sensor_objid[sensor_id_force];

    RCLCPP_INFO(logger_, "Registered FTS under fts_grav_comp_plugin with");
    RCLCPP_INFO(logger_, "\tname: '%s'", fts_data.sensor_name.c_str());
    RCLCPP_INFO(logger_, "\tfts_site: '%s'", mj_id2name(model, mjOBJ_SITE, fts_data.fts_site_id));
    RCLCPP_INFO(logger_, "\tcog_mass: '%0.3f'", fts_data.cog_mass);
    RCLCPP_INFO(logger_, "\tcog_pos: '%0.3f, %0.3f, %0.3f'", fts_data.cog_pos[0], fts_data.cog_pos[1],
                fts_data.cog_pos[2]);
    RCLCPP_INFO(logger_, "\tcog_site: '%s'", mj_id2name(model, mjOBJ_SITE, fts_data.cog_site_id));
    fts_.push_back(fts_data);
  }
  return true;
}

bool FtsGravCompPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());

  if (!register_fts(model))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to register force torque sensors.");
    return false;
  }
  if (fts_.empty())
  {
    return true;
  }

  fts_wrench_publisher_ = node_->create_publisher<geometry_msgs::msg::Wrench>("fts_wrench", 10);
  fts_comped_wrench_publisher_ = node_->create_publisher<geometry_msgs::msg::Wrench>("fts_comped_wrench", 10);

  // Initialize the last publish time
  last_publish_time_ = node_->get_clock()->now();

  RCLCPP_INFO(logger_, "FtsGravCompPlugin initialized. Publishing to topic 'fts_wrench' every %.3f second(s).",
              publish_period_.seconds());

  return true;
}

void FtsGravCompPlugin::update(const mjModel* model, mjData* data)
{
  auto current_time = node_->get_clock()->now();
  auto elapsed = current_time - last_publish_time_;

  // TODO - this is just here for troubleshooting for now, will come out in final version
  if (elapsed >= publish_period_)
  {
    const mjtNum* sensordata_force = data->sensordata + fts_[0].sensor_adr_force;
    const mjtNum* sensordata_torque = data->sensordata + fts_[0].sensor_adr_torque;
    // everything must be negated to handle the way force and torque sensors are set up
    fts_[0].wrench.force.x = -sensordata_force[0];
    fts_[0].wrench.force.y = -sensordata_force[1];
    fts_[0].wrench.force.z = -sensordata_force[2];
    fts_[0].wrench.torque.x = -sensordata_torque[0];
    fts_[0].wrench.torque.y = -sensordata_torque[1];
    fts_[0].wrench.torque.z = -sensordata_torque[2];

    fts_wrench_publisher_->publish(fts_[0].wrench);
  }

  // run gravity compensation for each force torque sensor
  for (auto& fts : fts_)
  {
    // Get the FTS data locally
    mjtNum* sensordata_force = data->sensordata + fts.sensor_adr_force;
    mjtNum* sensordata_torque = data->sensordata + fts.sensor_adr_torque;
    // pre-processed force and torque
    mjtNum raw_force_in_fts_frame[3];
    mjtNum raw_torque_in_fts_frame[3];
    mju_copy3(raw_force_in_fts_frame, sensordata_force);
    mju_copy3(raw_torque_in_fts_frame, sensordata_torque);
    // gravity compensated force and torque
    mjtNum comped_force_in_fts_frame[3];
    mjtNum comped_torque_in_fts_frame[3];

    // Step 1 - transform cog pos to be about the fts frame
    // part a: transform cog pos to world: pos_world = R_cog_frame * pos_in_cog_frame + pos_in_cog_frame
    mjtNum cog_pos_world[3];
    mju_mulMatVec3(cog_pos_world, data->site_xmat + 9 * fts.cog_site_id, fts.cog_pos);
    mju_addTo3(cog_pos_world, data->site_xpos + 3 * fts.cog_site_id);

    // part b: transform to FTS frame: pos_in_fts_frame = R_ft_frame^T * (pos_world - pos_ft_frame)
    mjtNum pos_relative[3];
    mjtNum cog_pos_in_fts_frame[3];
    mju_sub3(pos_relative, cog_pos_world, data->site_xpos + 3 * fts.fts_site_id);
    mju_mulMatTVec3(cog_pos_in_fts_frame, data->site_xmat + 9 * fts.fts_site_id, pos_relative);

    // Step 2 - get gravity into the FTS frame
    mjtNum gravity_in_fts_frame[3];
    mjtNum weight_in_fts_frame[3];
    mjtNum weight_torque_in_fts_frame[3];
    mju_mulMatTVec3(gravity_in_fts_frame, data->site_xmat + 9 * fts.fts_site_id, model->opt.gravity);
    mju_scl3(weight_in_fts_frame, gravity_in_fts_frame, fts.cog_mass);

    // Step 3 - subtract out forces and torques due to weight of the end effector
    // subtract out weight of gravity from the wrench value (add bc mujoco has negative values for these)
    mju_add3(comped_force_in_fts_frame, raw_force_in_fts_frame, weight_in_fts_frame);
    // calculate torque, and subtract out torque from the torque value (add bc mujoco has negative values for these)
    mju_cross(weight_torque_in_fts_frame, cog_pos_in_fts_frame, weight_in_fts_frame);
    mju_add3(comped_torque_in_fts_frame, raw_torque_in_fts_frame, weight_torque_in_fts_frame);

    // copy the data back to the FTS data
    mju_copy3(sensordata_force, comped_force_in_fts_frame);
    mju_copy3(sensordata_torque, comped_torque_in_fts_frame);
  }

  // TODO - this is just here for troubleshooting for now, will come out in final version
  if (elapsed >= publish_period_)
  {
    const mjtNum* sensordata_force = data->sensordata + fts_[0].sensor_adr_force;
    const mjtNum* sensordata_torque = data->sensordata + fts_[0].sensor_adr_torque;
    // everything must be negated to handle the way force and torque sensors are set up
    fts_[0].wrench.force.x = -sensordata_force[0];
    fts_[0].wrench.force.y = -sensordata_force[1];
    fts_[0].wrench.force.z = -sensordata_force[2];
    fts_[0].wrench.torque.x = -sensordata_torque[0];
    fts_[0].wrench.torque.y = -sensordata_torque[1];
    fts_[0].wrench.torque.z = -sensordata_torque[2];

    fts_comped_wrench_publisher_->publish(fts_[0].wrench);

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
