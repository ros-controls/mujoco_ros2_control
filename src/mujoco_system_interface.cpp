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

#include "mujoco_ros2_simulation/mujoco_system_interface.hpp"
#include "array_safety.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

using namespace std::chrono_literals;

// constants
const double kSyncMisalign = 0.1;        // maximum misalignment before re-sync (simulation seconds)
const double kSimRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;           // load error string length

using Seconds = std::chrono::duration<double>;

namespace mujoco_ros2_simulation
{
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// Clamps v to the lo or high value
double clamp(double v, double lo, double hi)
{
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir()
{
  constexpr char kPathSep = '/';
  const char* path = "/proc/self/exe";

  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success)
    {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      auto written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size)
      {
        realpath.get()[written] = '\0';
        success = true;
      }
      else if (written == -1)
      {
        if (errno == EINVAL)
        {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      }
      else
      {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();

  if (realpath.empty())
  {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i)
  {
    if (realpath.c_str()[i] == kPathSep)
    {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries()
{
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin)
  {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i)
    {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  const std::string sep = "/";

  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty())
  {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i)
        {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//------------------------------------------- simulation
//-------------------------------------------

const char* Diverged(int disableflags, const mjData* d)
{
  if (disableflags & mjDSBL_AUTORESET)
  {
    for (mjtWarning w : { mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS })
    {
      if (d->warning[w].number > 0)
      {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim)
{
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0])
  {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename) > 4 && !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                                                     mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
  {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew)
    {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  }
  else
  {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0])
    {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n')
      {
        loadError[error_length - 1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew)
  {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0])
  {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25)
  {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

MujocoSystemInterface::MujocoSystemInterface() = default;

MujocoSystemInterface::~MujocoSystemInterface()
{
  // Stop camera rendering loop
  if (cameras_)
  {
    cameras_->close();
  }

  // Stop lidar sensor loop
  if (lidar_sensors_)
  {
    lidar_sensors_->close();
  }

  // Stop ROS
  if (executor_)
  {
    executor_->cancel();
    executor_thread_.join();
  }

  // If sim_ is created and running, clean shut it down
  if (sim_)
  {
    sim_->exitrequest.store(true);
    sim_->run = false;

    if (physics_thread_.joinable())
    {
      physics_thread_.join();
    }
    if (ui_thread_.joinable())
    {
      ui_thread_.join();
    }
  }

  // Cleanup data and the model, if they haven't been
  if (mj_data_)
  {
    mj_deleteData(mj_data_);
  }
  if (mj_model_)
  {
    mj_deleteModel(mj_model_);
  }
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  system_info_ = info;

  // Helper function to get parameters in hardware info.
  auto get_parameter = [&](const std::string& key) -> std::optional<std::string> {
    if (auto it = info.hardware_parameters.find(key); it != info.hardware_parameters.end())
    {
      return it->second;
    }
    return std::nullopt;
  };

  // Load the model path from hardware parameters
  const auto model_path_maybe = get_parameter("mujoco_model");
  if (!model_path_maybe.has_value())
  {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Missing 'mujoco_model' in <hardware_parameters>.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  model_path_ = model_path_maybe.value();

  RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"), "Loading 'mujoco_model' from: " << model_path_);

  // Pull the initial speed factor from the hardware parameters, if present
  sim_speed_factor_ = std::stod(get_parameter("sim_speed_factor").value_or("-1"));
  if (sim_speed_factor_ > 0)
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                       "Running the simulation at " << sim_speed_factor_ * 100.0 << " percent speed");
  }
  else
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"), "No sim_speed set, using the setting from the UI");
  }

  // Pull the camera publish rate out of the info, if present, otherwise default to 5 hz.
  const auto camera_publish_rate = std::stod(get_parameter("camera_publish_rate").value_or("5.0"));
  // Pull the lidar publish rate out of the info, if present, otherwise default to 5 hz.
  const auto lidar_publish_rate = std::stod(get_parameter("lidar_publish_rate").value_or("5.0"));

  // We essentially reconstruct the 'simulate.cc::main()' function here, and
  // launch a Simulate object with all necessary rendering process/options
  // attached.

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  // Retain scope
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  // There is a timing issue here as the rendering context must be attached to
  // the executing thread, but we require the simulation to be available on
  // init. So we spawn the sim in the rendering thread prior to proceeding with
  // initialization.
  auto sim_ready = std::make_shared<std::promise<void>>();
  std::future<void> sim_ready_future = sim_ready->get_future();

  // Launch the UI loop in the background
  ui_thread_ = std::thread([this, sim_ready]() {
    sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam_, &opt_, &pert_,
                                          /* is_passive = */ false);
    // Notify sim that we are ready
    sim_ready->set_value();

    RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Starting the mujoco rendering thread...");
    // Blocks until terminated
    sim_->RenderLoop();
  });

  if (sim_ready_future.wait_for(2s) == std::future_status::timeout)
  {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Timed out waiting to start simulation rendering!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // We maintain a pointer to the mutex so that we can lock from here, too.
  // Is this a terrible idea? Maybe, but it lets us use their libraries as is...
  sim_mutex_ = &sim_->mtx;

  // Load the model and data prior to hw registration and starting the physics thread
  sim_->LoadMessage(model_path_.c_str());
  mj_model_ = LoadModel(model_path_.c_str(), *sim_);
  if (!mj_model_)
  {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Mujoco failed to load '%s'", model_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mj_data_ = mj_makeData(mj_model_);
  }
  if (!mj_data_)
  {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Could not allocate mjData for '%s'", model_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Pull joint and sensor information
  register_joints(info);
  register_sensors(info);
  set_initial_pose();

  // Construct and start the ROS node spinning
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  mujoco_node_ = std::make_shared<rclcpp::Node>("mujoco_node");
  executor_->add_node(mujoco_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  // Time publisher will be pushed from the physics_thread_
  clock_publisher_ = mujoco_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

  // Ready cameras
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Initializing cameras...");
  cameras_ = std::make_unique<MujocoCameras>(mujoco_node_, sim_mutex_, mj_data_, mj_model_, camera_publish_rate);
  cameras_->register_cameras(info);

  // Configure Lidar sensors
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Initializing lidar...");
  lidar_sensors_ = std::make_unique<MujocoLidar>(mujoco_node_, sim_mutex_, mj_data_, mj_model_, lidar_publish_rate);
  if (!lidar_sensors_->register_lidar(info))
  {
    RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Failed to initialize lidar, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // When the interface is activated, we start the physics engine.
  physics_thread_ = std::thread([this]() {
    // Load the simulation and do an initial forward pass
    RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Starting the mujoco physics thread...");
    sim_->Load(mj_model_, mj_data_, model_path_.c_str());
    // lock the sim mutex
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      mj_forward(mj_model_, mj_data_);
    }
    // Blocks until terminated
    PhysicsLoop();
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> new_state_interfaces;

  // Joint state interfaces
  for (auto& joint : joint_states_)
  {
    // Add state interfaces for joint hardware.
    if (auto it = joint_hw_info_.find(joint.name); it != joint_hw_info_.end())
    {
      for (const auto& state_if : it->second.state_interfaces)
      {
        if (state_if.name == hardware_interface::HW_IF_POSITION)
        {
          new_state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position);
        }
        else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          new_state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity);
        }
        else if (state_if.name == hardware_interface::HW_IF_EFFORT)
        {
          new_state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort);
        }
      }
    }
  }

  // Add state interfaces for fts sensors
  for (auto& sensor : ft_sensor_data_)
  {
    if (auto it = sensors_hw_info_.find(sensor.name); it != sensors_hw_info_.end())
    {
      for (const auto& state_if : it->second.state_interfaces)
      {
        if (state_if.name == "force.x")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.force.data.x());
        }
        else if (state_if.name == "force.y")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.force.data.y());
        }
        else if (state_if.name == "force.z")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.force.data.z());
        }
        else if (state_if.name == "torque.x")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.torque.data.x());
        }
        else if (state_if.name == "torque.y")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.torque.data.y());
        }
        else if (state_if.name == "torque.z")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.torque.data.z());
        }
      }
    }
  }

  // Add state interfaces for IMU sensors
  for (auto& sensor : imu_sensor_data_)
  {
    if (auto it = sensors_hw_info_.find(sensor.name); it != sensors_hw_info_.end())
    {
      for (const auto& state_if : it->second.state_interfaces)
      {
        if (state_if.name == "orientation.x")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.x());
        }
        else if (state_if.name == "orientation.y")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.y());
        }
        else if (state_if.name == "orientation.z")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.z());
        }
        else if (state_if.name == "orientation.w")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation.data.w());
        }
        else if (state_if.name == "angular_velocity.x")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity.data.x());
        }
        else if (state_if.name == "angular_velocity.y")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity.data.y());
        }
        else if (state_if.name == "angular_velocity.z")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity.data.z());
        }
        else if (state_if.name == "linear_acceleration.x")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration.data.x());
        }
        else if (state_if.name == "linear_acceleration.y")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration.data.y());
        }
        else if (state_if.name == "linear_acceleration.z")
        {
          new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration.data.z());
        }
        // Add covariance interfaces, these aren't currently used but some controllers require them.
        // TODO: Is there mujoco covariance data we could use?
        else if (state_if.name.find("orientation_covariance") == 0)
        {
          // Convert the index from the end of the string, this doesn't really matter yet
          size_t idx = std::stoul(state_if.name.substr(23));
          if (idx < sensor.orientation_covariance.size())
          {
            new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.orientation_covariance[idx]);
          }
        }
        else if (state_if.name.find("angular_velocity_covariance") == 0)
        {
          // Convert the index from the end of the string, this doesn't really matter yet
          size_t idx = std::stoul(state_if.name.substr(28));
          if (idx < sensor.angular_velocity_covariance.size())
          {
            new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.angular_velocity_covariance[idx]);
          }
        }
        else if (state_if.name.find("linear_acceleration_covariance") == 0)
        {
          // Convert the index from the end of the string, this doesn't really matter yet
          size_t idx = std::stoul(state_if.name.substr(31));
          if (idx < sensor.linear_acceleration_covariance.size())
          {
            new_state_interfaces.emplace_back(sensor.name, state_if.name, &sensor.linear_acceleration_covariance[idx]);
          }
        }
      }
    }
  }

  return new_state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> new_command_interfaces;

  // Joint command interfaces
  for (auto& joint : joint_states_)
  {
    // Add command interfaces for joint hardware.
    if (auto it = joint_hw_info_.find(joint.name); it != joint_hw_info_.end())
    {
      for (const auto& command_if : it->second.command_interfaces)
      {
        if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
        {
          new_command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &joint.position_command);
        }
        else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
        {
          new_command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &joint.velocity_command);
        }
        else if (command_if.name == hardware_interface::HW_IF_EFFORT)
        {
          new_command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &joint.effort_command);
        }
      }
    }
  }

  return new_command_interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
              "Activating MuJoCo hardware interface and starting Simulate threads...");

  // Start camera and sensor rendering loops
  cameras_->init();
  lidar_sensors_->init();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocoSystemInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
              "Deactivating MuJoCo hardware interface and shutting down Simulate...");

  // TODO: Should we shut mujoco things down here or in the destructor?

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MujocoSystemInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                   const std::vector<std::string>& stop_interfaces)
{
  auto update_joint_interface = [this](const std::string& interface_name, bool enabled) {
    size_t delimiter_pos = interface_name.find('/');
    if (delimiter_pos == std::string::npos)
    {
      RCLCPP_ERROR(rclcpp::get_logger("MujocoSystemInterface"), "Invalid interface name format: %s",
                   interface_name.c_str());
      return;
    }

    std::string joint_name = interface_name.substr(0, delimiter_pos);
    std::string interface_type = interface_name.substr(delimiter_pos + 1);

    // Find the JointState in the vector
    auto joint_it = std::find_if(joint_states_.begin(), joint_states_.end(),
                                 [&joint_name](const JointState& joint) { return joint.name == joint_name; });

    if (joint_it == joint_states_.end())
    {
      RCLCPP_WARN(rclcpp::get_logger("MujocoSystemInterface"), "Joint %s not found in joint_states_",
                  joint_name.c_str());
      return;
    }

    if (enabled)
    {
      // Only one type of control mode can be active at a time, so stop everything first then enable the
      // requested command interface.
      joint_it->is_position_control_enabled = false;
      joint_it->is_velocity_control_enabled = false;
      joint_it->is_effort_control_enabled = false;

      if (interface_type == hardware_interface::HW_IF_POSITION)
      {
        joint_it->is_position_control_enabled = true;
        RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
                    "Joint %s: position control enabled (velocity, effort disabled)", joint_name.c_str());
      }
      else if (interface_type == hardware_interface::HW_IF_VELOCITY)
      {
        joint_it->is_velocity_control_enabled = true;
        RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
                    "Joint %s: velocity control enabled (position, effort disabled)", joint_name.c_str());
      }
      else if (interface_type == hardware_interface::HW_IF_EFFORT)
      {
        joint_it->is_effort_control_enabled = true;
        RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
                    "Joint %s: effort control enabled (position, velocity disabled)", joint_name.c_str());
      }
    }
    else
    {
      if (interface_type == hardware_interface::HW_IF_POSITION)
      {
        joint_it->is_position_control_enabled = false;
      }
      else if (interface_type == hardware_interface::HW_IF_VELOCITY)
      {
        joint_it->is_velocity_control_enabled = false;
      }
      else if (interface_type == hardware_interface::HW_IF_EFFORT)
      {
        joint_it->is_effort_control_enabled = false;
      }
      RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Joint %s: %s control disabled", joint_name.c_str(),
                  interface_type.c_str());
    }
  };

  // Disable stopped interfaces
  for (const auto& interface : stop_interfaces)
  {
    update_joint_interface(interface, false);
  }

  // Enable started interfaces
  for (const auto& interface : start_interfaces)
  {
    update_joint_interface(interface, true);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::read(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/)
{
  std::lock_guard<std::recursive_mutex> lock(*sim_mutex_);

  // Joint states
  for (auto& joint_state : joint_states_)
  {
    joint_state.position = mj_data_->qpos[joint_state.mj_pos_adr];
    joint_state.velocity = mj_data_->qvel[joint_state.mj_vel_adr];
    joint_state.effort = mj_data_->qfrc_actuator[joint_state.mj_vel_adr];
  }

  // IMU Sensor data
  for (auto& data : imu_sensor_data_)
  {
    data.orientation.data.w() = mj_data_->sensordata[data.orientation.mj_sensor_index];
    data.orientation.data.x() = mj_data_->sensordata[data.orientation.mj_sensor_index + 1];
    data.orientation.data.y() = mj_data_->sensordata[data.orientation.mj_sensor_index + 2];
    data.orientation.data.z() = mj_data_->sensordata[data.orientation.mj_sensor_index + 3];

    data.angular_velocity.data.x() = mj_data_->sensordata[data.angular_velocity.mj_sensor_index];
    data.angular_velocity.data.y() = mj_data_->sensordata[data.angular_velocity.mj_sensor_index + 1];
    data.angular_velocity.data.z() = mj_data_->sensordata[data.angular_velocity.mj_sensor_index + 2];

    data.linear_acceleration.data.x() = mj_data_->sensordata[data.linear_acceleration.mj_sensor_index];
    data.linear_acceleration.data.y() = mj_data_->sensordata[data.linear_acceleration.mj_sensor_index + 1];
    data.linear_acceleration.data.z() = mj_data_->sensordata[data.linear_acceleration.mj_sensor_index + 2];
  }

  // FT Sensor data
  for (auto& data : ft_sensor_data_)
  {
    data.force.data.x() = -mj_data_->sensordata[data.force.mj_sensor_index];
    data.force.data.y() = -mj_data_->sensordata[data.force.mj_sensor_index + 1];
    data.force.data.z() = -mj_data_->sensordata[data.force.mj_sensor_index + 2];

    data.torque.data.x() = -mj_data_->sensordata[data.torque.mj_sensor_index];
    data.torque.data.y() = -mj_data_->sensordata[data.torque.mj_sensor_index + 1];
    data.torque.data.z() = -mj_data_->sensordata[data.torque.mj_sensor_index + 2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  std::lock_guard<std::recursive_mutex> lock(*sim_mutex_);

  // Update mimic joints
  for (auto& joint_state : joint_states_)
  {
    if (joint_state.is_mimic)
    {
      joint_state.position_command =
          joint_state.mimic_multiplier * joint_states_.at(joint_state.mimicked_joint_index).position_command;
      joint_state.velocity_command =
          joint_state.mimic_multiplier * joint_states_.at(joint_state.mimicked_joint_index).velocity_command;
      joint_state.effort_command =
          joint_state.mimic_multiplier * joint_states_.at(joint_state.mimicked_joint_index).effort_command;
    }
  }

  // Joint commands
  // TODO: Support command limits. For now those ranges can be limited in the mujoco actuators themselves.
  for (auto& joint_state : joint_states_)
  {
    if (joint_state.mj_actuator_id == -1)
    {
      // Skip joints without actuators
      continue;
    }

    if (joint_state.is_position_control_enabled)
    {
      mj_data_->ctrl[joint_state.mj_actuator_id] = joint_state.position_command;
    }
    else if (joint_state.is_velocity_control_enabled)
    {
      mj_data_->ctrl[joint_state.mj_actuator_id] = joint_state.velocity_command;
    }
    else if (joint_state.is_effort_control_enabled)
    {
      mj_data_->ctrl[joint_state.mj_actuator_id] = joint_state.effort_command;
    }
  }

  return hardware_interface::return_type::OK;
}

void MujocoSystemInterface::register_joints(const hardware_interface::HardwareInfo& hardware_info)
{
  joint_states_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    int mujoco_joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
    if (mujoco_joint_id == -1)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                          "Failed to find joint in mujoco model, joint name: " << joint.name);
      continue;
    }

    // Try to locate the matching actuator id for the joint, if available
    int mujoco_actuator_id = -1;
    for (int i = 0; i < mj_model_->nu; ++i)
    {
      // If it is the correct type and matches the joint id, we're done
      if (mj_model_->actuator_trntype[i] == mjTRN_JOINT && mj_model_->actuator_trnid[2 * i] == mujoco_joint_id)
      {
        mujoco_actuator_id = i;
        break;
      }
    }

    // Is no mapping was found, try fallback to looking for an actuator with the same name as the joint
    mujoco_actuator_id = mujoco_actuator_id == -1 ? mj_name2id(mj_model_, mjtObj::mjOBJ_ACTUATOR, joint.name.c_str()) :
                                                    mujoco_actuator_id;

    if (mujoco_actuator_id == -1)
    {
      // This isn't a failure the joint just won't be controllable
      RCLCPP_WARN_STREAM(rclcpp::get_logger("MujocoSystemInterface"), "No actuator found for joint: " << joint.name);
    }

    // Add to the joint hw information map
    joint_hw_info_.insert(std::make_pair(joint.name, joint));

    // save information in joint_states_ variable
    JointState joint_state;
    joint_state.name = joint.name;
    joint_state.mj_joint_type = mj_model_->jnt_type[mujoco_joint_id];
    joint_state.mj_pos_adr = mj_model_->jnt_qposadr[mujoco_joint_id];
    joint_state.mj_vel_adr = mj_model_->jnt_dofadr[mujoco_joint_id];
    joint_state.mj_actuator_id = mujoco_actuator_id;

    joint_states_.at(joint_index) = joint_state;
    JointState& last_joint_state = joint_states_.at(joint_index);

    // check if mimicked
    if (joint.parameters.find("mimic") != joint.parameters.end())
    {
      const auto mimicked_joint = joint.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(hardware_info.joints.begin(), hardware_info.joints.end(),
                                                  [&mimicked_joint](const hardware_interface::ComponentInfo& info) {
                                                    return info.name == mimicked_joint;
                                                  });
      if (mimicked_joint_it == hardware_info.joints.end())
      {
        throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }
      last_joint_state.is_mimic = true;
      last_joint_state.mimicked_joint_index = std::distance(hardware_info.joints.begin(), mimicked_joint_it);

      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        last_joint_state.mimic_multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      else
      {
        last_joint_state.mimic_multiplier = 1.0;
      }
    }

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo& interface_info) {
      if (!interface_info.initial_value.empty())
      {
        double value = std::stod(interface_info.initial_value);
        return value;
      }
      else
      {
        return 0.0;
      }
    };

    // Set initial values if they are set in the info
    for (const auto& state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        last_joint_state.position = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        last_joint_state.velocity = get_initial_value(state_if);
      }
      else if (state_if.name == hardware_interface::HW_IF_EFFORT)
      {
        last_joint_state.effort = get_initial_value(state_if);
      }
    }

    // command interfaces
    // overwrite joint limit with min/max value
    for (const auto& command_if : joint.command_interfaces)
    {
      // If available, always default to position control at the start
      if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
      {
        last_joint_state.is_position_control_enabled = true;
        last_joint_state.position_command = last_joint_state.position;
      }
      else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
      {
        last_joint_state.is_velocity_control_enabled = true;
        last_joint_state.velocity_command = last_joint_state.velocity;
      }
      else if (command_if.name.find(hardware_interface::HW_IF_EFFORT) != std::string::npos)
      {
        last_joint_state.is_effort_control_enabled = true;
        last_joint_state.effort_command = last_joint_state.effort;
      }
    }
  }
}

void MujocoSystemInterface::register_sensors(const hardware_interface::HardwareInfo& hardware_info)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    const std::string sensor_name = sensor.name;

    if (sensor.parameters.count("mujoco_type") == 0)
    {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                         "Not adding hardware interface for sensor in ros2_control xacro: " << sensor_name);
      continue;
    }
    const auto mujoco_type = sensor.parameters.at("mujoco_type");

    // If there is a specific sensor name provided we use that, otherwise we assume the mujoco model's
    // sensor is named identically to the ros2_control hardware interface's.
    std::string mujoco_sensor_name;
    if (sensor.parameters.count("mujoco_sensor_name") == 0)
    {
      mujoco_sensor_name = sensor_name;
    }
    else
    {
      mujoco_sensor_name = sensor.parameters.at("mujoco_sensor_name");
    }

    RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                       "Adding sensor named: " << sensor_name << ", of type: " << mujoco_type
                                               << ", mapping to the MJCF sensor: " << mujoco_sensor_name);

    // Add to the sensor hw information map
    sensors_hw_info_.insert(std::make_pair(sensor_name, sensor));

    if (mujoco_type == "fts")
    {
      FTSensorData sensor_data;
      sensor_data.name = sensor_name;
      sensor_data.force.name = mujoco_sensor_name + "_force";
      sensor_data.torque.name = mujoco_sensor_name + "_torque";

      int force_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.force.name.c_str());
      int torque_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.torque.name.c_str());

      if (force_sensor_id == -1 || torque_sensor_id == -1)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                            "Failed to find force/torque sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.force.mj_sensor_index = mj_model_->sensor_adr[force_sensor_id];
      sensor_data.torque.mj_sensor_index = mj_model_->sensor_adr[torque_sensor_id];

      ft_sensor_data_.push_back(sensor_data);
    }

    else if (mujoco_type == "imu")
    {
      IMUSensorData sensor_data;
      sensor_data.name = sensor_name;
      sensor_data.orientation.name = mujoco_sensor_name + "_quat";
      sensor_data.angular_velocity.name = mujoco_sensor_name + "_gyro";
      sensor_data.linear_acceleration.name = mujoco_sensor_name + "_accel";

      // Initialize to all zeros as we do not use these yet.
      sensor_data.orientation_covariance.resize(9, 0.0);
      sensor_data.angular_velocity_covariance.resize(9, 0.0);
      sensor_data.linear_acceleration_covariance.resize(9, 0.0);

      int quat_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.orientation.name.c_str());
      int gyro_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.angular_velocity.name.c_str());
      int accel_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.linear_acceleration.name.c_str());

      if (quat_id == -1 || gyro_id == -1 || accel_id == -1)
      {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                            "Failed to find IMU sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.orientation.mj_sensor_index = mj_model_->sensor_adr[quat_id];
      sensor_data.angular_velocity.mj_sensor_index = mj_model_->sensor_adr[gyro_id];
      sensor_data.linear_acceleration.mj_sensor_index = mj_model_->sensor_adr[accel_id];

      imu_sensor_data_.push_back(sensor_data);
    }
    else
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("MujocoSystemInterface"),
                          "Invalid mujoco_type passed to the mujoco hardware interface: " << mujoco_type);
    }
  }
}

void MujocoSystemInterface::set_initial_pose()
{
  for (auto& joint_state : joint_states_)
  {
    mj_data_->qpos[joint_state.mj_pos_adr] = joint_state.position;
  }
}

// simulate in background thread (while rendering in main thread)
void MujocoSystemInterface::PhysicsLoop()
{
  // cpu-sim synchronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim_->exitrequest.load())
  {
    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery
    //  life
    if (sim_->run && sim_->busywait)
    {
      std::this_thread::yield();
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex during the update
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);

      // run only if model is present
      if (mj_model_)
      {
        // running
        if (sim_->run)
        {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = mj_data_->time - syncSim;

          // Ordinarily the speed factor for the simulation is pulled from the sim UI. However, this is
          // overridable by setting the "sim_speed_factor" parameter in the hardware info.
          // If that parameter is set, then we ignore whatever slowdown has been requested from the UI.
          double speedFactor = sim_speed_factor_ < 0 ? (100.0 / sim_->percentRealTime[sim_->real_time_index]) :
                                                       (1.0 / sim_speed_factor_);

          // misalignment condition: distance from target sim time is bigger
          // than syncmisalign
          bool misaligned = std::abs(Seconds(elapsedCPU).count() / speedFactor - elapsedSim) > kSyncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim_->speed_changed)
          {
            // re-sync
            syncCPU = startCPU;
            syncSim = mj_data_->time;
            sim_->speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(mj_model_, mj_data_);
            const char* message = Diverged(mj_model_->opt.disableflags, mj_data_);
            if (message)
            {
              sim_->run = 0;
              mju::strcpy_arr(sim_->load_error, message);
            }
            else
            {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else
          {
            bool measured = false;
            mjtNum prevSim = mj_data_->time;

            double refreshTime = kSimRefreshFraction / sim_->refresh_rate;

            // step while sim lags behind cpu and within refreshTime.
            while (Seconds((mj_data_->time - syncSim) * speedFactor) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
            {
              // measure slowdown before first step
              if (!measured && elapsedSim)
              {
                sim_->measured_slowdown = std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }
              // inject noise
              sim_->InjectNoise();

              // call mj_step
              mj_step(mj_model_, mj_data_);
              const char* message = Diverged(mj_model_->opt.disableflags, mj_data_);
              if (message)
              {
                sim_->run = 0;
                mju::strcpy_arr(sim_->load_error, message);
              }
              else
              {
                stepped = true;
              }

              // break if reset
              if (mj_data_->time < prevSim)
              {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped)
          {
            sim_->AddToHistory();
          }
        }

        // paused
        else
        {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(mj_model_, mj_data_);
          sim_->speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>

    // Publish the clock
    publish_clock();
  }
}

void MujocoSystemInterface::publish_clock()
{
  auto sim_time = mj_data_->time;
  int32_t sim_time_sec = static_cast<int32_t>(std::floor(sim_time));
  uint32_t sim_time_nanosec = static_cast<uint32_t>((sim_time - sim_time_sec) * 1e9);
  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);

  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time_ros;
  clock_publisher_->publish(sim_time_msg);
}

}  // namespace mujoco_ros2_simulation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_simulation::MujocoSystemInterface, hardware_interface::SystemInterface);
