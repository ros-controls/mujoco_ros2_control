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

#include "mujoco_ros2_control/mujoco_lidar.hpp"
#include "mujoco_ros2_control/utils.hpp"

namespace mujoco_ros2_control
{

/**
 * Construct a LidarConfig object given a string sensor name and hardware_info object to parse.
 */
std::optional<LidarConfig> get_lidar_config(const hardware_interface::HardwareInfo& hardware_info, const std::string& name)
{
  const auto sensor_info_maybe = get_sensor_from_info(hardware_info, name);
  if (!sensor_info_maybe.has_value())
  {
    return std::nullopt;
  }
  const auto sensor_info = sensor_info_maybe.value();

  auto get_parameter = [&](const std::string& key) -> std::optional<std::string> {
    if (auto it = sensor_info.parameters.find(key); it != sensor_info.parameters.end())
    {
      return it->second;
    }
    return std::nullopt;
  };

  auto frame_name = get_parameter("frame_name");
  auto min_angle = get_parameter("min_angle");
  auto max_angle = get_parameter("max_angle");
  auto angle_increment = get_parameter("angle_increment");
  auto laserscan_topic = get_parameter("laserscan_topic");
  auto range_min = get_parameter("range_min");
  auto range_max = get_parameter("range_max");

  // If any required parameters are missing fire off an error.
  if (!frame_name || !angle_increment || !min_angle || !max_angle)
  {
    return std::nullopt;
  }

  // Otherwise construct and return a new LidarData object
  LidarConfig lidar_sensor;
  lidar_sensor.frame_name = *frame_name;
  lidar_sensor.min_angle = std::stod(*min_angle);
  lidar_sensor.max_angle = std::stod(*max_angle);
  lidar_sensor.angle_increment = std::stod(*angle_increment);
  

  lidar_sensor.laserscan_topic = laserscan_topic.has_value() ? laserscan_topic.value() : "/scan";
  lidar_sensor.range_min = range_min.has_value() ? std::stod(range_min.value()) : 0.0;
  lidar_sensor.range_max = range_max.has_value() ? std::stod(range_max.value()) : 1000.0;
  lidar_sensor.num_rangefinders = static_cast<int>((lidar_sensor.max_angle - lidar_sensor.min_angle)/lidar_sensor.angle_increment) + 1;

  // Configure the static parameters of the laserscan message
  lidar_sensor.laser_scan_msg.header.frame_id = lidar_sensor.frame_name;
  lidar_sensor.laser_scan_msg.time_increment = 0.0;  // Does this matter?
  lidar_sensor.laser_scan_msg.angle_min = static_cast<float>(lidar_sensor.min_angle);
  lidar_sensor.laser_scan_msg.angle_max = static_cast<float>(lidar_sensor.max_angle);
  lidar_sensor.laser_scan_msg.angle_increment = static_cast<float>(lidar_sensor.angle_increment);
  lidar_sensor.laser_scan_msg.range_min = static_cast<float>(lidar_sensor.range_min);
  lidar_sensor.laser_scan_msg.range_max = static_cast<float>(lidar_sensor.range_max);
  lidar_sensor.laser_scan_msg.ranges.resize(lidar_sensor.num_rangefinders);
  lidar_sensor.laser_scan_msg.intensities.resize(0);

  return lidar_sensor;
}

MujocoLidar::MujocoLidar(rclcpp::Node::SharedPtr node, std::recursive_mutex* sim_mutex, mjData* mujoco_data,
                         mjModel* mujoco_model, double lidar_publish_rate)
  : node_(node)
  , sim_mutex_(sim_mutex)
  , mj_data_(mujoco_data)
  , mj_model_(mujoco_model)
  , lidar_publish_rate_(lidar_publish_rate)
{
}

bool MujocoLidar::register_lidar(const hardware_interface::HardwareInfo& hardware_info)
{
  for (int i = 0; i < mj_model_->nsensor; ++i)
  {
    // Skip sensors not controlled by plugin.
    if (mj_model_->sensor_type[i] != mjtSensor::mjSENS_PLUGIN)
    {
      continue;
    }

    // Grab the name of the sensor, which is required.
    const auto sensor_name_maybe = mj_id2name(mj_model_, mjtObj::mjOBJ_SENSOR, i);
    
    if (sensor_name_maybe == nullptr)
    {
      RCLCPP_WARN(node_->get_logger(), "Cannot find a name for lidar sensor at index: '%d' skipping!", i);
      continue;
    }
    
    RCLCPP_INFO(node_->get_logger(), "Adding lidar sensor: '%s', idx: %d", sensor_name_maybe, lidar_config_.sensor_id);

    auto new_data_maybe = get_lidar_config(hardware_info, sensor_name_maybe);
    if (!new_data_maybe.has_value())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse required configuration from ros2_control xacro: '%s'",
                    sensor_name_maybe);
      return false;
    }


    lidar_config_ = new_data_maybe.value();

    // Setup remaining msg params and publisher for the sensor
    lidar_config_.name = sensor_name_maybe;
    lidar_config_.sensor_id = i;
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(lidar_config_.laserscan_topic, 1);

    // We may someday want to compute this on the fly, but since everything is fixed this should be fine for now.
    lidar_config_.laser_scan_msg.scan_time = 1.0f / static_cast<float>(lidar_publish_rate_);

    // Note that we have added the sensor
    RCLCPP_INFO(node_->get_logger(), "Adding lidar sensor: '%s', idx: %d", lidar_config_.name.c_str(), lidar_config_.sensor_id);
    RCLCPP_INFO(node_->get_logger(), "         frame_name: '%s'", lidar_config_.frame_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "          min_angle: %f", lidar_config_.min_angle);
    RCLCPP_INFO(node_->get_logger(), "          max_angle: %f", lidar_config_.max_angle);
    RCLCPP_INFO(node_->get_logger(), "    angle_increment: %f", lidar_config_.angle_increment);
    RCLCPP_INFO(node_->get_logger(), "          range_min: %f", lidar_config_.range_min);
    RCLCPP_INFO(node_->get_logger(), "          range_max: %f", lidar_config_.range_max);

  }

  return true;
}

void MujocoLidar::init()
{
  // Start the rendering thread process
  publish_lidar_ = true;
  rendering_thread_ = std::thread(&MujocoLidar::update_loop, this);
}

void MujocoLidar::close()
{
  publish_lidar_ = false;
  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
  }
}

void MujocoLidar::update_loop()
{
  // Setup container for lidar data.
  mj_lidar_data_.resize(mj_model_->nsensordata);

  RCLCPP_INFO(node_->get_logger(), "Starting the lidar processing loop, publishing at %f Hz", lidar_publish_rate_);

  rclcpp::Rate rate(lidar_publish_rate_);
  while (rclcpp::ok() && publish_lidar_)
  {
    update();
    rate.sleep();
  }
}

void MujocoLidar::update()
{
  
  // Step 1: Lock the sim and copy only the sensordata
  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    std::memcpy(mj_lidar_data_.data(), mj_data_->sensordata, mj_lidar_data_.size() * sizeof(mjtNum));
  }

  for(int i = 0; i < lidar_config_.num_rangefinders; ++i)
  {
    lidar_config_.laser_scan_msg.ranges[i] = static_cast<float>(mj_data_->sensordata[lidar_config_.sensor_id + i]);
  }

  // Apply range limits to the copied data
  std::transform(lidar_config_.laser_scan_msg.ranges.begin(), lidar_config_.laser_scan_msg.ranges.end(),
                  lidar_config_.laser_scan_msg.ranges.begin(),
                  [&](auto range) { return (range < lidar_config_.range_min || range > lidar_config_.range_max) ? -1.0 : range; });


  lidar_config_.laser_scan_msg.header.stamp = node_->now();
  scan_pub_->publish(lidar_config_.laser_scan_msg);
}

}  // namespace mujoco_ros2_control
