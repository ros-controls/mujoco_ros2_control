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

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <tuple>

#include <mujoco/mujoco.h>
#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>
#include "ament_index_cpp/version.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mujoco_3d_lidar_plugin.hpp"

class Mujoco3dLidarPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    // Find and load the lidar plugin via the ament index, this is essentially what happens
    // in the system interface.
#if AMENT_INDEX_CPP_VERSION_GTE(1, 13, 2)
    auto resource = ament_index_cpp::get_resource("mujoco_plugins", "mujoco_3d_lidar");
    if (resource.resourcePath.has_value())
    {
      const std::string plugin_dir = (resource.resourcePath.value() / resource.contents).string();
      mj_loadPluginLibrary((plugin_dir + "/libmujoco_3d_lidar.so").c_str());
    }
#else
    std::string content, prefix;
    if (ament_index_cpp::get_resource("mujoco_plugins", "mujoco_3d_lidar", content, &prefix))
    {
      const std::string plugin_dir = prefix + "/" + content;
      mj_loadPluginLibrary((plugin_dir + "/libmujoco_3d_lidar.so").c_str());
    }
#endif

    node_ = rclcpp::Node::make_shared("test_lidar_plugin_node");
    plugin_node_ = node_->create_sub_node("lidar_plugin");
    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    if (executor_)
    {
      executor_->cancel();
    }
    if (spin_thread_.joinable())
    {
      spin_thread_.join();
    }

    executor_.reset();
    plugin_node_.reset();
    node_.reset();

    if (data_)
    {
      mj_deleteData(data_);
      data_ = nullptr;
    }
    if (model_)
    {
      mj_deleteModel(model_);
      model_ = nullptr;
    }

    for (const auto& path : temp_files_)
    {
      if (std::filesystem::exists(path))
      {
        std::filesystem::remove(path);
      }
    }
  }

  void load_model(const std::string& xml)
  {
    auto path = "/tmp/test_camera_plugin_" + std::to_string(temp_files_.size()) + ".xml";
    {
      std::ofstream f(path);
      f << xml;
    }
    temp_files_.push_back(path);

    char error[1000] = "";
    model_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << "mj_loadXML failed: " << error;
    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
  }

  // Name of a parameter of the cloud_3d sensor, as the plugin declares it
  std::string param_name(const std::string& name) const
  {
    return "mujoco_plugins." + plugin_node_->get_sub_namespace() + ".cloud_3d." + name;
  }

  // A 3D lidar above a plane: the rays below the horizon hit the plane, the others have no return
  static std::string lidar_3d_model()
  {
    return R"(<?xml version="1.0"?>
<mujoco model="lidar_3d_model">
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="robot" pos="0 0 0.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <site name="lidar_3d_site" pos="0 0 0.15"/>
    </body>
  </worldbody>
  <extension>
    <plugin plugin="mujoco.plugin.lidar">
      <instance name="cloud_3d">
        <config key="resolution" value="8 4"/>
        <config key="azimuth_range" value="-0.5 0.5"/>
        <config key="elevation_range" value="-0.3 0.3"/>
        <config key="max_range" value="10.0"/>
        <config key="min_range" value="0.05"/>
        <config key="update_rate" value="100.0"/>
        <config key="async" value="0"/>
      </instance>
    </plugin>
  </extension>
  <sensor>
    <plugin instance="cloud_3d" name="cloud_3d" objtype="site" objname="lidar_3d_site"/>
  </sensor>
</mujoco>
)";
  }

  // Steps the simulation until the plugin publishes a cloud on /points
  sensor_msgs::msg::PointCloud2::SharedPtr receive_cloud(mujoco_ros2_control_plugins::Mujoco3dLidarPlugin& plugin)
  {
    std::atomic<bool> got_cloud{ false };
    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud;
    auto cloud_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", 1, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          last_cloud = msg;
          got_cloud = true;
        });
    for (int i = 0; i < 60 && !got_cloud; ++i)
    {
      mj_step(model_, data_);
      plugin.update_write(model_, data_);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return last_cloud;
  }

  // Checks the name, offset and type of the fields of a cloud
  static void expect_fields(const sensor_msgs::msg::PointCloud2& cloud,
                            const std::vector<std::tuple<std::string, uint32_t, uint8_t>>& fields, uint32_t point_step)
  {
    ASSERT_EQ(cloud.fields.size(), fields.size());
    for (size_t i = 0; i < fields.size(); ++i)
    {
      EXPECT_EQ(cloud.fields[i].name, std::get<0>(fields[i]));
      EXPECT_EQ(cloud.fields[i].offset, std::get<1>(fields[i]));
      EXPECT_EQ(cloud.fields[i].datatype, std::get<2>(fields[i]));
      EXPECT_EQ(cloud.fields[i].count, 1u);
    }
    EXPECT_EQ(cloud.point_step, point_step);
    EXPECT_EQ(cloud.row_step, cloud.point_step * cloud.width);
    EXPECT_EQ(cloud.data.size(), static_cast<size_t>(cloud.row_step) * cloud.height);
  }

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr plugin_node_;
  mjModel* model_{ nullptr };
  mjData* data_{ nullptr };
  std::vector<std::string> temp_files_;
};

TEST_F(Mujoco3dLidarPluginTest, InitSucceedsWithNoLidar)
{
  load_model(R"(<?xml version="1.0"?>
<mujoco model="empty_model">
  <worldbody>
    <body name="box" pos="0 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.05 0.05 0.05"/>
    </body>
  </worldbody>
</mujoco>
)");
  mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));
  plugin.update_write(model_, data_);
  plugin.cleanup();
}

TEST_F(Mujoco3dLidarPluginTest, Test2dLidar)
{
  load_model(R"(<?xml version="1.0"?>
<mujoco model="lidar_2d_model">
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="robot" pos="0 0 0.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <site name="lidar_site" pos="0 0 0.15"/>
    </body>
  </worldbody>
  <extension>
    <plugin plugin="mujoco.plugin.lidar">
      <instance name="scan_2d">
        <config key="resolution" value="24 1"/>
        <config key="azimuth_range" value="-1.57 1.57"/>
        <config key="elevation_range" value="0.0"/>
        <config key="max_range" value="10.0"/>
        <config key="min_range" value="0.05"/>
        <config key="update_rate" value="100.0"/>
        <config key="async" value="0"/>
      </instance>
    </plugin>
  </extension>
  <sensor>
    <plugin instance="scan_2d" name="scan_2d" objtype="site" objname="lidar_site"/>
  </sensor>
</mujoco>
)");

  mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
  EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));

  // Create subscriber to default topic
  std::atomic<bool> got_scan{ false };
  sensor_msgs::msg::LaserScan::SharedPtr last_scan;
  auto scan_sub = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, [&](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan = msg;
        got_scan = true;
      });

  // Step the sim so the plugin computes sensordata, then call update
  for (int i = 0; i < 20; ++i)
  {
    mj_step(model_, data_);
    plugin.update_write(model_, data_);
    if (got_scan)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(got_scan) << "No LaserScan received";
  if (last_scan)
  {
    EXPECT_EQ(last_scan->ranges.size(), 24u);
    EXPECT_FLOAT_EQ(last_scan->angle_min, -1.57f);
    EXPECT_FLOAT_EQ(last_scan->angle_max, 1.57f);
  }

  plugin.cleanup();
}

TEST_F(Mujoco3dLidarPluginTest, Test3dLidarSyncAsync)
{
  // Just run this twice with the two different execution modes
  for (const auto& async_value : { "0", "1" })
  {
    // Logs which path failed, if it fails
    SCOPED_TRACE(std::string("async=") + async_value);

    load_model(R"(<?xml version="1.0"?>
<mujoco model="lidar_3d_model">
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="robot" pos="0 0 0.5">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom type="box" size="0.1 0.1 0.1"/>
      <site name="lidar_3d_site" pos="0 0 0.15"/>
    </body>
  </worldbody>
  <extension>
    <plugin plugin="mujoco.plugin.lidar">
      <instance name="cloud_3d">
        <config key="resolution" value="8 4"/>
        <config key="azimuth_range" value="-0.5 0.5"/>
        <config key="elevation_range" value="-0.3 0.3"/>
        <config key="max_range" value="10.0"/>
        <config key="min_range" value="0.05"/>
        <config key="update_rate" value="100.0"/>
        <config key="async" value=")" +
               std::string(async_value) + R"("/>
      </instance>
    </plugin>
  </extension>
  <sensor>
    <plugin instance="cloud_3d" name="cloud_3d" objtype="site" objname="lidar_3d_site"/>
  </sensor>
</mujoco>
)");

    mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
    EXPECT_TRUE(plugin.init(plugin_node_, model_, data_));

    // Create subscriber to default topic
    std::atomic<bool> got_cloud{ false };
    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud;
    auto cloud_sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", 1, [&](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          last_cloud = msg;
          got_cloud = true;
        });

    // Helps to make sure the async thread has time to trigger
    for (int i = 0; i < 60; ++i)
    {
      mj_step(model_, data_);
      plugin.update_write(model_, data_);
      if (got_cloud)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(got_cloud) << "No PointCloud2 received";
    if (last_cloud)
    {
      EXPECT_EQ(last_cloud->width, 8u);
      EXPECT_EQ(last_cloud->height, 4u);
    }

    plugin.cleanup();
  }
}

TEST_F(Mujoco3dLidarPluginTest, Test3dLidarPointFieldsXyz)
{
  load_model(lidar_3d_model());

  mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  auto cloud = receive_cloud(plugin);
  ASSERT_TRUE(cloud) << "No PointCloud2 received";

  using sensor_msgs::msg::PointField;
  expect_fields(*cloud,
                { { "x", 0, PointField::FLOAT32 }, { "y", 4, PointField::FLOAT32 }, { "z", 8, PointField::FLOAT32 } },
                12);
  EXPECT_FALSE(cloud->is_dense);

  plugin.cleanup();
}

TEST_F(Mujoco3dLidarPluginTest, Test3dLidarPointFieldsRobosenseVelodyne)
{
  for (const std::string layout : { "robosense", "velodyne" })
  {
    SCOPED_TRACE("point_fields=" + layout);
    load_model(lidar_3d_model());
    if (plugin_node_->has_parameter(param_name("point_fields")))
    {
      plugin_node_->set_parameter(rclcpp::Parameter(param_name("point_fields"), layout));
    }
    else
    {
      plugin_node_->declare_parameter(param_name("point_fields"), layout);
    }

    mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
    ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
    auto cloud = receive_cloud(plugin);
    ASSERT_TRUE(cloud) << "No PointCloud2 received";

    using sensor_msgs::msg::PointField;
    const bool robosense = layout == "robosense";
    expect_fields(*cloud,
                  { { "x", 0, PointField::FLOAT32 },
                    { "y", 4, PointField::FLOAT32 },
                    { "z", 8, PointField::FLOAT32 },
                    { "intensity", 12, PointField::FLOAT32 },
                    { "ring", 16, PointField::UINT16 },
                    { robosense ? "timestamp" : "time", robosense ? 24u : 20u,
                      robosense ? PointField::FLOAT64 : PointField::FLOAT32 } },
                  robosense ? 32 : 24);
    EXPECT_FALSE(cloud->is_dense);

    const double stamp = rclcpp::Time(cloud->header.stamp).seconds();
    sensor_msgs::PointCloud2ConstIterator<float> iterX(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterIntensity(*cloud, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iterRing(*cloud, "ring");
    int returns = 0;
    for (uint32_t i = 0; i < cloud->width * cloud->height; ++i, ++iterX, ++iterIntensity, ++iterRing)
    {
      // ring is the row of the scan, intensity 1 for a return and 0 otherwise
      EXPECT_EQ(*iterRing, i / cloud->width);
      EXPECT_FLOAT_EQ(*iterIntensity, std::isfinite(*iterX) ? 1.0f : 0.0f);
      returns += std::isfinite(*iterX) ? 1 : 0;
    }
    // Rays below the horizon hit the plane, the others have no return
    EXPECT_GT(returns, 0);
    EXPECT_LT(returns, static_cast<int>(cloud->width * cloud->height));

    // Every point is taken at the stamp of the scan
    if (robosense)
    {
      for (sensor_msgs::PointCloud2ConstIterator<double> it(*cloud, "timestamp"); it != it.end(); ++it)
      {
        EXPECT_NEAR(*it, stamp, 1e-6);
      }
    }
    else
    {
      for (sensor_msgs::PointCloud2ConstIterator<float> it(*cloud, "time"); it != it.end(); ++it)
      {
        EXPECT_FLOAT_EQ(*it, 0.0f);
      }
    }

    plugin.cleanup();
    mj_deleteData(data_);
    data_ = nullptr;
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

TEST_F(Mujoco3dLidarPluginTest, Test3dLidarPointFieldsOuster)
{
  load_model(lidar_3d_model());
  plugin_node_->declare_parameter(param_name("point_fields"), std::string("ouster"));

  mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
  ASSERT_TRUE(plugin.init(plugin_node_, model_, data_));
  auto cloud = receive_cloud(plugin);
  ASSERT_TRUE(cloud) << "No PointCloud2 received";

  using sensor_msgs::msg::PointField;
  expect_fields(*cloud,
                { { "x", 0, PointField::FLOAT32 },
                  { "y", 4, PointField::FLOAT32 },
                  { "z", 8, PointField::FLOAT32 },
                  { "intensity", 12, PointField::FLOAT32 },
                  { "t", 16, PointField::UINT32 },
                  { "reflectivity", 20, PointField::UINT16 },
                  { "ring", 22, PointField::UINT16 },
                  { "ambient", 24, PointField::UINT16 },
                  { "range", 28, PointField::UINT32 } },
                32);
  // Rays without a return are zeros, not NaN
  EXPECT_TRUE(cloud->is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> iterX(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iterY(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iterZ(*cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iterIntensity(*cloud, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iterT(*cloud, "t");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> iterRing(*cloud, "ring");
  sensor_msgs::PointCloud2ConstIterator<uint32_t> iterRange(*cloud, "range");
  int returns = 0;
  for (uint32_t i = 0; i < cloud->width * cloud->height;
       ++i, ++iterX, ++iterY, ++iterZ, ++iterIntensity, ++iterT, ++iterRing, ++iterRange)
  {
    EXPECT_TRUE(std::isfinite(*iterX));
    EXPECT_EQ(*iterRing, i / cloud->width);
    EXPECT_EQ(*iterT, 0u);
    if (*iterRange > 0)
    {
      // range in millimetres matches the point
      const double dist = std::sqrt(*iterX * *iterX + *iterY * *iterY + *iterZ * *iterZ);
      EXPECT_NEAR(*iterRange, dist * 1000.0, 1.0);
      EXPECT_FLOAT_EQ(*iterIntensity, 1.0f);
      ++returns;
    }
    else
    {
      EXPECT_FLOAT_EQ(*iterX, 0.0f);
      EXPECT_FLOAT_EQ(*iterY, 0.0f);
      EXPECT_FLOAT_EQ(*iterZ, 0.0f);
      EXPECT_FLOAT_EQ(*iterIntensity, 0.0f);
    }
  }
  EXPECT_GT(returns, 0);
  EXPECT_LT(returns, static_cast<int>(cloud->width * cloud->height));

  plugin.cleanup();
}

TEST_F(Mujoco3dLidarPluginTest, Test3dLidarInvalidPointFieldsFails)
{
  load_model(lidar_3d_model());
  plugin_node_->declare_parameter(param_name("point_fields"), std::string("xyzirt"));

  mujoco_ros2_control_plugins::Mujoco3dLidarPlugin plugin;
  EXPECT_FALSE(plugin.init(plugin_node_, model_, data_));
  plugin.cleanup();
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
