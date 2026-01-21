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

#include "mujoco_ros2_control/mujoco_system_interface.hpp"
#include "array_safety.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>

#include <tinyxml2.h>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>
#if !ROS_DISTRO_HUMBLE
#include <hardware_interface/helpers.hpp>
#endif
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lodepng.h"

#include "control_toolbox/pid.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

using namespace std::chrono_literals;

// constants
const double kSyncMisalign = 0.1;        // maximum misalignment before re-sync (simulation seconds)
const double kSimRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;           // load error string length

using Seconds = std::chrono::duration<double>;

namespace
{
std::optional<std::string> get_hardware_parameter(const hardware_interface::HardwareInfo& hardware_info,
                                                  const std::string& key)
{
  if (auto it = hardware_info.hardware_parameters.find(key); it != hardware_info.hardware_parameters.end())
  {
    return it->second;
  }
  return std::nullopt;
}

std::string get_hardware_parameter_or(const hardware_interface::HardwareInfo& hardware_info, const std::string& key,
                                      const std::string& default_value)
{
  if (auto it = hardware_info.hardware_parameters.find(key); it != hardware_info.hardware_parameters.end())
  {
    return it->second;
  }
  return default_value;
}
}  // namespace
namespace mujoco_ros2_control
{
template <typename T>
void add_items(std::vector<T>& vector, const std::vector<T>& items)
{
#if ROS_DISTRO_HUMBLE
  for (const auto& item : items)
  {
    if (std::find(vector.begin(), vector.end(), item) == vector.end())
    {
      vector.push_back(item);
    }
  }
#else
  for (const auto& item : items)
  {
    ros2_control::add_item(vector, item);
  }
#endif
}
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

/**
 * No-op UI adapter to support running the drivers in a headless environment.
 */
class HeadlessAdapter : public mj::PlatformUIAdapter
{
public:
  HeadlessAdapter() = default;
  ~HeadlessAdapter() override = default;

  std::pair<double, double> GetCursorPosition() const override
  {
    return { 0.0, 0.0 };
  }
  double GetDisplayPixelsPerInch() const override
  {
    return 96.0;
  }
  std::pair<int, int> GetFramebufferSize() const override
  {
    return { 800, 600 };
  }
  std::pair<int, int> GetWindowSize() const override
  {
    return { 800, 600 };
  }
  bool IsGPUAccelerated() const override
  {
    return false;
  }
  void PollEvents() override
  {
  }
  void SetClipboardString(const char* /*text*/) override
  {
  }
  void SetVSync(bool /*enabled*/) override
  {
  }
  void SetWindowTitle(const char* /*title*/) override
  {
  }
  bool ShouldCloseWindow() const override
  {
    return false;
  }
  void SwapBuffers() override
  {
  }
  void ToggleFullscreen() override
  {
  }

  bool IsLeftMouseButtonPressed() const override
  {
    return false;
  }
  bool IsMiddleMouseButtonPressed() const override
  {
    return false;
  }
  bool IsRightMouseButtonPressed() const override
  {
    return false;
  }

  bool IsAltKeyPressed() const override
  {
    return false;
  }
  bool IsCtrlKeyPressed() const override
  {
    return false;
  }
  bool IsShiftKeyPressed() const override
  {
    return false;
  }

  bool IsMouseButtonDownEvent(int /*act*/) const override
  {
    return false;
  }
  bool IsKeyDownEvent(int /*act*/) const override
  {
    return false;
  }

  int TranslateKeyCode(int /*key*/) const override
  {
    return 0;
  }
  mjtButton TranslateMouseButton(int /*button*/) const override
  {
    return mjBUTTON_NONE;
  }

  bool RefreshMjrContext(const mjModel* /*m*/, int /*fontscale*/) override
  {
    return false;
  }
};

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

  std::string real_path = [&]() -> std::string {
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

  if (real_path.empty())
  {
    return "";
  }

  for (std::size_t i = real_path.size() - 1; i > 0; --i)
  {
    if (real_path.c_str()[i] == kPathSep)
    {
      return real_path.substr(0, i);
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

mjModel* loadModelFromFile(const char* file, mj::Simulate& sim)
{
  mjModel* mnew = 0;

  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // load and compile
  char loadError[kErrorLength] = "";
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
      auto error_length = mju::strlen_arr(loadError);
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

mjModel* loadModelFromTopic(rclcpp::Node::SharedPtr node, const std::string& topic_name)
{
  mjModel* mnew = 0;
  std::string robot_description;

  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.reliable().transient_local().keep_last(1);
  RCLCPP_INFO(node->get_logger(), "Trying to get the mujoco model from topic");

  // Try to get mujoco_model via topic
  auto mujoco_model_sub = node->create_subscription<std_msgs::msg::String>(
      topic_name, qos_profile, [&](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg->data.empty() && robot_description.empty())
          robot_description = msg->data;
      });

  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(120);

  while (robot_description.empty() && rclcpp::ok())
  {
    auto now = std::chrono::steady_clock::now();

    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "Waiting for /mujoco_robot_description...");

    if (now - start > timeout)
    {
      RCLCPP_WARN(node->get_logger(), "Timeout waiting for /mujoco_robot_description topic. Aborting...");
      exit(1);
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  if (!robot_description.empty())
  {
    // Load Mujoco model
    char error[1000] = "Could not load XML model";

    mjSpec* spec = nullptr;
    spec = mj_parseXMLString(robot_description.c_str(), nullptr, error, 1000);
    mnew = mj_compile(spec, nullptr);

    if (!mnew)
    {
      const char* myerr = mjs_getError(spec);
      RCLCPP_INFO(node->get_logger(), "Error %s", myerr);
      RCLCPP_FATAL(node->get_logger(), "Failed to compile MuJoCo model: %s", error);
      mj_deleteSpec(spec);
    }
    mj_deleteSpec(spec);
    RCLCPP_INFO(node->get_logger(), "Model body count: %d", mnew->nbody);
    RCLCPP_INFO(node->get_logger(), "Model geom count: %d", mnew->ngeom);
  }
  return mnew;
}

mjModel* LoadModel(const char* file, const std::string& topic, mj::Simulate& sim, rclcpp::Node::SharedPtr node)
{
  // Try to get the mujoco model from URDF.
  // If it is not available, create a subscription and listen for the model on a topic.

  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // load model from path if the filename is not empty
  if (filename[0])
  {
    return loadModelFromFile(file, sim);
  }
  // Try to get the mujoco model from topic
  return loadModelFromTopic(node, topic);
}

ActuatorType getActuatorType(const mjModel* mj_model, int mujoco_actuator_id)
{
  // Returns the MuJoCo actuator type based on the actuator's bias settings.
  ActuatorType actuator_type = ActuatorType::UNKNOWN;
  int biastype = mj_model->actuator_biastype[mujoco_actuator_id];
  const int NBias = 10;
  const mjtNum* biasprm = mj_model->actuator_biasprm + mujoco_actuator_id * NBias;

  if (biastype == mjBIAS_NONE)
  {
    actuator_type = ActuatorType::MOTOR;
  }
  else if (biastype == mjBIAS_AFFINE && biasprm[1] != 0)
  {
    actuator_type = ActuatorType::POSITION;
  }
  else if (biastype == mjBIAS_AFFINE && biasprm[1] == 0 && biasprm[2] != 0)
  {
    actuator_type = ActuatorType::VELOCITY;
  }
  else
  {
    // If none of the standard bias patterns match, classify as a custom actuator
    actuator_type = ActuatorType::CUSTOM;
  }

  return actuator_type;
}

/**
 * @brief Get the MuJoCo actuator ID based on a name. First this method looks for a joint that matches the passed name,
 * and finds the actuator attached to it. If this doesn't exist, it will then look for the actuator with the passed name.
 * @param actuator_name The name of the actuator.
 * @param mj_model Pointer to the MuJoCo model.
 * @return The actuator ID if found, otherwise -1.
 */
int get_actuator_id(const std::string& actuator_name, const mjModel* mj_model)
{
  int mujoco_actuator_id = mj_name2id(mj_model, mjtObj::mjOBJ_JOINT, actuator_name.c_str());
  if (mujoco_actuator_id == -1)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("MujocoSystemInterface"), "Failed to find the actuator : '%s' in the MuJoCo model",
                 actuator_name.c_str());
  }

  // Try to locate the matching actuator id for the joint, if available
  for (int i = 0; i < mj_model->nu; ++i)
  {
    // If it is the correct type and matches the joint id, we're done
    if (mj_model->actuator_trntype[i] == mjTRN_JOINT && mj_model->actuator_trnid[2 * i] == mujoco_actuator_id)
    {
      mujoco_actuator_id = i;
      break;
    }
  }

  // Is no mapping was found, try fallback to looking for an actuator with the same name as the joint
  mujoco_actuator_id = mujoco_actuator_id == -1 ? mj_name2id(mj_model, mjtObj::mjOBJ_ACTUATOR, actuator_name.c_str()) :
                                                  mujoco_actuator_id;
  return mujoco_actuator_id;
}

/**
 * @brief Get the corresponding actuator name for a given joint name using transmissions.
 * @param joint_name The name of the joint.
 * @param hardware_info The hardware information containing transmissions.
 * @param mj_model Pointer to the MuJoCo model.
 * @return The corresponding actuator name if found, otherwise returns the joint name.
 */
std::string get_joint_actuator_name(const std::string& joint_name,
                                    const hardware_interface::HardwareInfo& hardware_info, const mjModel* mj_model)
{
  std::string actuator_name = joint_name;  // Default to joint name

  for (const auto& transmission : hardware_info.transmissions)
  {
    for (const auto& joint : transmission.joints)
    {
      if (joint.name == joint_name)
      {
        if (get_actuator_id(joint_name, mj_model) != -1)
        {
          return joint_name;  // Direct match found
        }
        // replace "joint" with "actuator" for the corresponding role
        const std::string corresponding_actuator_role = std::regex_replace(joint.role, std::regex("joint"), "actuator");
        for (const auto& actuator : transmission.actuators)
        {
          if (actuator.role == corresponding_actuator_role)
          {
            RCLCPP_DEBUG(rclcpp::get_logger("MujocoSystemInterface"),
                         "Mapped joint '%s' to actuator '%s' based on role '%s'", joint_name.c_str(),
                         actuator.name.c_str(), corresponding_actuator_role.c_str());
            return actuator.name;
          }
        }
        RCLCPP_WARN(rclcpp::get_logger("MujocoSystemInterface"),
                    "No matching actuator found for joint '%s' with role '%s'. Using joint name as actuator name.",
                    joint_name.c_str(), joint.role.c_str());
        break;
      }
    }
  }

  return actuator_name;
}

/**
 * @brief Orders available interfaces based on a desired order.
 *
 * This function takes a list of available interfaces and a desired order,
 * returning a new list where the interfaces are arranged according to the
 * desired order. Any interfaces not specified in the desired order are appended
 * at the end in their original order.
 *
 * @param available_interfaces A vector of available interface names.
 * @param desired_order A vector specifying the desired order of interface names.
 * @return A vector of interface names ordered according to the desired order.
 */
std::vector<std::string> get_interfaces_in_order(const std::vector<std::string>& available_interfaces,
                                                 const std::vector<std::string>& desired_order)
{
  std::vector<std::string> ordered_interfaces;
  for (const auto& interface : desired_order)
  {
    if (std::find(available_interfaces.begin(), available_interfaces.end(), interface) != available_interfaces.end())
    {
      ordered_interfaces.push_back(interface);
    }
  }
  mujoco_ros2_control::add_items(ordered_interfaces, available_interfaces);
  return ordered_interfaces;
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

  // If sim_ is created and running, clean shut it down
  // We do this first to ensure that no other threads are accessing the model/data
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

  // Stop ROS
  if (executor_)
  {
    executor_->cancel();
    executor_thread_.join();
  }

  // Remove all transmission instances
  transmission_instances_.clear();
  mujoco_actuator_data_.clear();
  urdf_joint_data_.clear();

  // Cleanup data and the model, if they haven't been
  if (mj_data_)
  {
    mj_deleteData(mj_data_);
  }
  if (mj_data_control_)
  {
    mj_deleteData(mj_data_control_);
  }
  if (mj_model_)
  {
    mj_deleteModel(mj_model_);
  }
}

hardware_interface::CallbackReturn
// after humble switches from HardwareInfo to HardwareComponentInterfaceParams. This keeps it backwards compatible
// between the two distros
#if ROS_DISTRO_HUMBLE
MujocoSystemInterface::on_init(const hardware_interface::HardwareInfo& params)
#else
MujocoSystemInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
#endif
{
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load the model path from hardware parameters
  const auto model_path_maybe = get_hardware_parameter(get_hardware_info(), "mujoco_model");
  if (!model_path_maybe.has_value())
  {
    RCLCPP_INFO(get_logger(), "Parameter 'mujoco_model' not found in URDF.");
    model_path_.clear();
  }
  else
  {
    model_path_ = model_path_maybe.value();
    // trim the trailing and leading whitespaces
    model_path_.erase(0, model_path_.find_first_not_of(" \t\n\r\f\v"));
    model_path_.erase(model_path_.find_last_not_of(" \t\n\r\f\v") + 1);
    const std::filesystem::path path_to_file(model_path_);
    if (!std::filesystem::exists(path_to_file))
    {
      RCLCPP_FATAL(get_logger(), "MuJoCo model file '%s' does not exist!", model_path_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(get_logger(), "Loading 'mujoco_model' from: '%s'", model_path_.c_str());
  }

  // Pull the initial speed factor from the hardware parameters, if present
  sim_speed_factor_ = std::stod(get_hardware_parameter(get_hardware_info(), "sim_speed_factor").value_or("-1"));
  if (sim_speed_factor_ > 0)
  {
    RCLCPP_INFO(get_logger(), "Running the simulation at %.2f percent speed", sim_speed_factor_ * 100.0);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "No sim_speed set, using the setting from the UI");
  }

  // Pull the camera publish rate out of the info, if present, otherwise default to 5 hz.
  const auto camera_publish_rate =
      std::stod(get_hardware_parameter(get_hardware_info(), "camera_publish_rate").value_or("5.0"));
  // Pull the lidar publish rate out of the info, if present, otherwise default to 5 hz.
  const auto lidar_publish_rate =
      std::stod(get_hardware_parameter(get_hardware_info(), "lidar_publish_rate").value_or("5.0"));

  // Check for headless mode
  bool headless =
      hardware_interface::parse_bool(get_hardware_parameter(get_hardware_info(), "headless").value_or("false"));
  RCLCPP_INFO_EXPRESSION(get_logger(), headless, "Running in HEADLESS mode.");

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

  if (headless)
  {
    sim_ = std::make_unique<mj::Simulate>(std::make_unique<HeadlessAdapter>(), &cam_, &opt_, &pert_,
                                          /* is_passive = */ false);
    // Notify sim that we are ready
    sim_ready->set_value();
  }
  else
  {
    // Launch the UI loop in the background
    ui_thread_ = std::thread([this, sim_ready]() {
      sim_ = std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam_, &opt_, &pert_,
                                            /* is_passive = */ false);

      // Add ros2 control icon for the taskbar
      std::string icon_location =
          ament_index_cpp::get_package_share_directory("mujoco_ros2_control") + "/resources/mujoco_logo.png";
      std::vector<unsigned char> image;
      unsigned width, height;
      unsigned error = lodepng::decode(image, width, height, icon_location);

      // Only process the icon if we successfully loaded it. Otherwise, just proceed without
      if (error)
      {
        RCLCPP_WARN_STREAM(get_logger(), "LodePNG error " << error << ": " << lodepng_error_text(error)
                                                          << ". Icon file not loaded: " << icon_location);
      }
      else
      {
        GLFWimage icon;
        icon.width = width;
        icon.height = height;
        icon.pixels = image.data();
        glfwSetWindowIcon(glfwGetCurrentContext(), 1, &icon);
      }

      // Set glfw window size to max size of the primary monitor
      const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
      glfwSetWindowSize(glfwGetCurrentContext(), mode->width, mode->height);

      // Hide UI panels programmatically
      sim_->ui0_enable = false;  // Hide left panel
      sim_->ui1_enable = false;  // Hide right panel

      // Notify sim that we are ready
      sim_ready->set_value();

      // Blocks until terminated
      RCLCPP_INFO(get_logger(), "Starting the mujoco rendering thread...");
      sim_->RenderLoop();
    });
  }

  if (sim_ready_future.wait_for(2s) == std::future_status::timeout)
  {
    RCLCPP_FATAL(get_logger(), "Timed out waiting to start simulation rendering!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // We maintain a pointer to the mutex so that we can lock from here, too.
  // Is this a terrible idea? Maybe, but it lets us use their libraries as is...
  sim_mutex_ = &sim_->mtx;

  // Load the model and data prior to hw registration and starting the physics thread
  sim_->LoadMessage(model_path_.c_str());

  // Construct and start the ROS node spinning
  /// The PIDs config file
  const auto pids_config_file = get_hardware_parameter(get_hardware_info(), "pids_config_file");
  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("use_sim_time", rclcpp::ParameterValue(true));
  if (pids_config_file.has_value())
  {
    std::string pids_config_file_path = pids_config_file.value();
    // trim the trailing and leading whitespaces
    pids_config_file_path.erase(0, pids_config_file_path.find_first_not_of(" \t\n\r\f\v"));
    pids_config_file_path.erase(pids_config_file_path.find_last_not_of(" \t\n\r\f\v") + 1);

    // Check if the file exists
    const std::filesystem::path path_to_file(pids_config_file_path);
    if (!std::filesystem::exists(path_to_file))
    {
      RCLCPP_FATAL(get_logger(), "PID config file '%s' does not exist!", pids_config_file->c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO_STREAM(get_logger(), "Loading PID config from file: " << pids_config_file.value());
    auto node_options_arguments = node_options.arguments();
    node_options_arguments.push_back(RCL_ROS_ARGS_FLAG);
    node_options_arguments.push_back(RCL_PARAM_FILE_FLAG);
    node_options_arguments.push_back(pids_config_file.value());
    node_options.arguments(node_options_arguments);
  }
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  mujoco_node_ = std::make_shared<rclcpp::Node>("mujoco_node", node_options);
  executor_->add_node(mujoco_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  const std::string mujoco_model_topic =
      get_hardware_parameter_or(get_hardware_info(), "mujoco_model_topic", "/mujoco_robot_description");
  mj_model_ = LoadModel(model_path_.c_str(), mujoco_model_topic, *sim_, mujoco_node_);
  if (!mj_model_)
  {
    RCLCPP_FATAL(get_logger(), "Mujoco failed to load the model");
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mj_data_ = mj_makeData(mj_model_);
    mj_data_control_ = mj_makeData(mj_model_);
  }
  if (!mj_data_ || !mj_data_control_)
  {
    RCLCPP_FATAL(get_logger(), "Could not allocate mjData for '%s'", model_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Time publisher will be pushed from the physics_thread_
  clock_publisher_ = mujoco_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  clock_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>>(clock_publisher_);

  actuator_state_publisher_ =
      mujoco_node_->create_publisher<sensor_msgs::msg::JointState>("/mujoco_actuators_states", 100);
  actuator_state_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>>(actuator_state_publisher_);

  // We explicitly check to make sure that all joints have names, otherwise stuff down the line won't work
  int num_joints_without_name = 0;
  for (int i = 0; i < mj_model_->njnt; ++i)
  {
    const char* joint_name = mj_id2name(mj_model_, mjtObj::mjOBJ_JOINT, i);
    if (!joint_name)
    {
      num_joints_without_name++;
    }
  }
  if (num_joints_without_name)
  {
    RCLCPP_FATAL(get_logger(), "%d joints in the mjcf don't have names. All joints must have names.",
                 num_joints_without_name);
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Register all MuJoCo actuators
  if (!register_mujoco_actuators())
  {
    RCLCPP_FATAL(get_logger(), "Failed to register MuJoCo actuators, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Check for free joint
  std::string odom_free_joint_name =
      get_hardware_parameter_or(get_hardware_info(), "odom_free_joint_name", "floating_base_joint");
  for (int i = 0; i < mj_model_->njnt; ++i)
  {
    const char* joint_name = mj_id2name(mj_model_, mjtObj::mjOBJ_JOINT, i);

    if (odom_free_joint_name == joint_name)
    {
      if (mj_model_->jnt_type[i] == mjJNT_FREE)
      {
        free_joint_id_ = i;
        free_joint_qpos_adr_ = mj_model_->jnt_qposadr[i];
        free_joint_qvel_adr_ = mj_model_->jnt_dofadr[i];
      }
      else
      {
        RCLCPP_FATAL(get_logger(),
                     "Unable to use joint '%s' to publish the floating base state since it is not a free joint.",
                     odom_free_joint_name.c_str());
        return hardware_interface::CallbackReturn::FAILURE;
      }
    }
  }

  if (free_joint_id_ != -1)
  {
    // Odometry publisher
    std::string odom_topic_name =
        get_hardware_parameter_or(get_hardware_info(), "odom_topic", "/simulator/floating_base_state");
    floating_base_publisher_ = mujoco_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);
    floating_base_realtime_publisher_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(floating_base_publisher_);

    floating_base_msg_.header.frame_id = "odom";  // TODO: Make configurable
    // Set child frame as the root link of the robot as the body attached to the free joint
    floating_base_msg_.child_frame_id =
        std::string(mj_id2name(mj_model_, mjtObj::mjOBJ_BODY, mj_model_->jnt_bodyid[free_joint_id_]));

    RCLCPP_INFO(
        get_logger(),
        "Publishing floating base odometry using the free joint : '%s' attached to the body '%s' on topic: '%s'",
        mj_id2name(mj_model_, mjtObj::mjOBJ_JOINT, free_joint_id_), floating_base_msg_.child_frame_id.c_str(),
        odom_topic_name.c_str());
  }

  // Pull joint and sensor information
  register_urdf_joints(get_hardware_info());
  register_sensors(get_hardware_info());
  if (!register_transmissions(get_hardware_info()))
  {
    RCLCPP_FATAL(get_logger(), "Failed to register transmissions, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }
  initialize_initial_positions(get_hardware_info());
  set_initial_pose();

  // Ready cameras
  RCLCPP_INFO(get_logger(), "Initializing cameras...");
  cameras_ = std::make_unique<MujocoCameras>(mujoco_node_, sim_mutex_, mj_data_, mj_model_, camera_publish_rate);
  cameras_->register_cameras(get_hardware_info());

  // Configure Lidar sensors
  RCLCPP_INFO(get_logger(), "Initializing lidar...");
  lidar_sensors_ = std::make_unique<MujocoLidar>(mujoco_node_, sim_mutex_, mj_data_, mj_model_, lidar_publish_rate);
  if (!lidar_sensors_->register_lidar(get_hardware_info()))
  {
    RCLCPP_INFO(get_logger(), "Failed to initialize lidar, exiting...");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  // Disable the rangefinder flag at startup so that we don't get the yellow lines.
  // We can still turn this on manually if desired.
  sim_->opt.flags[mjVIS_RANGEFINDER] = false;

#if !ROS_DISTRO_HUMBLE
  // Verify the update rate
  const mjtNum desired_timestep = 1.0 / static_cast<double>(get_hardware_info().rw_rate);
  const bool under_sampled = mj_model_->opt.timestep > desired_timestep;
  RCLCPP_WARN_EXPRESSION(
      get_logger(), under_sampled,
      "MuJoCo simulator frequency %lu Hz (timestep %.6f sec) is smaller than the controller manager's update rate %lu "
      "Hz. The simulation may be under-sampled and this means that there will be some discrepancies in the rate at "
      "which controllers update cycles run. Either increase the MuJoCo timestep or decrease the controller manager's "
      "update rate.",
      static_cast<unsigned long>(1.0 / mj_model_->opt.timestep), mj_model_->opt.timestep,
      static_cast<unsigned long>(get_hardware_info().rw_rate));
#endif

  // When the interface is activated, we start the physics engine.
  physics_thread_ = std::thread([this, headless]() {
    // Load the simulation and do an initial forward pass
    RCLCPP_INFO(get_logger(), "Starting the mujoco physics thread...");
    if (headless)
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      sim_->m_ = mj_model_;
      sim_->d_ = mj_data_;
      mju::strcpy_arr(sim_->filename, model_path_.c_str());
    }
    else
    {
      sim_->Load(mj_model_, mj_data_, model_path_.c_str());
    }
    // lock the sim mutex
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      mj_forward(mj_model_, mj_data_);
    }
    // Blocks until terminated
    PhysicsLoop();
  });

  actuator_state_msg_.name.clear();
  for (const auto& actuator : mujoco_actuator_data_)
  {
    actuator_state_msg_.name.push_back(actuator.joint_name);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> new_state_interfaces;

  // Joint state interfaces
  for (auto& joint : urdf_joint_data_)
  {
    // Add state interfaces for joint hardware.
    if (auto it = joint_hw_info_.find(joint.name); it != joint_hw_info_.end())
    {
      for (const auto& state_if : it->second.state_interfaces)
      {
        if (state_if.name == hardware_interface::HW_IF_POSITION)
        {
          new_state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                                            &joint.position_interface.state_);
        }
        else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          new_state_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                                            &joint.velocity_interface.state_);
        }
        else if (state_if.name == hardware_interface::HW_IF_EFFORT ||
                 state_if.name == hardware_interface::HW_IF_TORQUE || state_if.name == hardware_interface::HW_IF_FORCE)
        {
          new_state_interfaces.emplace_back(joint.name, state_if.name, &joint.effort_interface.state_);
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
  for (auto& joint : urdf_joint_data_)
  {
    // Add command interfaces for joint hardware.
    if (auto it = joint_hw_info_.find(joint.name); it != joint_hw_info_.end())
    {
      for (const auto& command_if : it->second.command_interfaces)
      {
        if (command_if.name.find(hardware_interface::HW_IF_POSITION) != std::string::npos)
        {
          new_command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_POSITION,
                                              &joint.position_interface.command_);
        }
        else if (command_if.name.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos)
        {
          new_command_interfaces.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY,
                                              &joint.velocity_interface.command_);
        }
        else if (command_if.name == hardware_interface::HW_IF_EFFORT ||
                 command_if.name == hardware_interface::HW_IF_TORQUE ||
                 command_if.name == hardware_interface::HW_IF_FORCE)
        {
          new_command_interfaces.emplace_back(joint.name, command_if.name, &joint.effort_interface.command_);
        }
      }
    }
  }

  return new_command_interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating MuJoCo hardware interface and starting Simulate threads...");

  // Start camera and sensor rendering loops
  cameras_->init();
  lidar_sensors_->init();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MujocoSystemInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating MuJoCo hardware interface and shutting down Simulate...");

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
      RCLCPP_ERROR(get_logger(), "Invalid interface name format: %s", interface_name.c_str());
      return;
    }

    std::string joint_name = interface_name.substr(0, delimiter_pos);
    std::string interface_type = interface_name.substr(delimiter_pos + 1);

    // Find the MuJoCoActuatorData in the vector
    auto joint_it = std::find_if(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                                 [&joint_name](const URDFJointData& joint) { return joint.name == joint_name; });

    if (joint_it == urdf_joint_data_.end())
    {
      RCLCPP_WARN(get_logger(), "Joint %s not found in urdf_joint_data_", joint_name.c_str());
      return;
    }

    const auto actuator_name = get_joint_actuator_name(joint_name, get_hardware_info(), mj_model_);

    auto actuator_it = std::find_if(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                                    [&actuator_name, this](const MuJoCoActuatorData& actuator) {
                                      return actuator.joint_name == actuator_name;
                                    });

    if (actuator_it == mujoco_actuator_data_.end())
    {
      RCLCPP_WARN(get_logger(), "Actuator %s not found in mujoco_actuator_data_", actuator_name.c_str());
      return;
    }
    if (actuator_it->actuator_type == ActuatorType::PASSIVE)
    {
      RCLCPP_WARN(get_logger(), "Actuator %s is passive and cannot be controlled.", actuator_name.c_str());
      return;
    }

    if (enabled)
    {
      // Only one type of control mode can be active at a time, so stop everything first then enable the
      // requested command interface.

      joint_it->is_position_control_enabled = false;
      joint_it->is_velocity_control_enabled = false;
      joint_it->is_effort_control_enabled = false;

      actuator_it->is_position_control_enabled = false;
      actuator_it->is_velocity_control_enabled = false;
      actuator_it->is_effort_control_enabled = false;
      actuator_it->is_position_pid_control_enabled = false;
      actuator_it->is_velocity_pid_control_enabled = false;

      if (interface_type == hardware_interface::HW_IF_POSITION)
      {
        actuator_it->is_position_control_enabled = (actuator_it->pos_pid == nullptr);
        actuator_it->is_position_pid_control_enabled = (actuator_it->pos_pid != nullptr);
        joint_it->is_position_control_enabled = true;
        RCLCPP_INFO(get_logger(), "Joint %s: position control enabled (velocity, effort disabled)", joint_name.c_str());
      }
      else if (interface_type == hardware_interface::HW_IF_VELOCITY)
      {
        actuator_it->is_velocity_control_enabled = (actuator_it->vel_pid == nullptr);
        actuator_it->is_velocity_pid_control_enabled = (actuator_it->vel_pid != nullptr);
        joint_it->is_velocity_control_enabled = true;
        RCLCPP_INFO(get_logger(), "Joint %s: velocity control enabled (position, effort disabled)", joint_name.c_str());
      }
      else if (interface_type == hardware_interface::HW_IF_EFFORT ||
               interface_type == hardware_interface::HW_IF_TORQUE || interface_type == hardware_interface::HW_IF_FORCE)
      {
        actuator_it->is_effort_control_enabled = true;
        joint_it->is_effort_control_enabled = true;
        RCLCPP_INFO(get_logger(), "Joint %s: %s control enabled (position, velocity disabled)", joint_name.c_str(),
                    interface_type.c_str());
      }
    }
    else
    {
      if (interface_type == hardware_interface::HW_IF_POSITION)
      {
        actuator_it->is_position_control_enabled = false;
        actuator_it->is_position_pid_control_enabled = false;
        joint_it->is_position_control_enabled = false;
      }
      else if (interface_type == hardware_interface::HW_IF_VELOCITY)
      {
        actuator_it->is_velocity_control_enabled = false;
        actuator_it->is_velocity_pid_control_enabled = false;
        joint_it->is_velocity_control_enabled = false;
      }
      else if (interface_type == hardware_interface::HW_IF_EFFORT ||
               interface_type == hardware_interface::HW_IF_TORQUE || interface_type == hardware_interface::HW_IF_FORCE)
      {
        actuator_it->is_effort_control_enabled = false;
        joint_it->is_effort_control_enabled = false;
      }
      RCLCPP_INFO(get_logger(), "Joint %s: %s control disabled", joint_name.c_str(), interface_type.c_str());
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

hardware_interface::return_type MujocoSystemInterface::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  // Joint states
  actuator_state_msg_.header.stamp = time;
  actuator_state_msg_.position.clear();
  actuator_state_msg_.velocity.clear();
  actuator_state_msg_.effort.clear();
  for (auto& actuator_state : mujoco_actuator_data_)
  {
    actuator_state.position_interface.state_ = mj_data_control_->qpos[actuator_state.mj_pos_adr];
    actuator_state.velocity_interface.state_ = mj_data_control_->qvel[actuator_state.mj_vel_adr];
    actuator_state.effort_interface.state_ = mj_data_control_->qfrc_actuator[actuator_state.mj_vel_adr];
    actuator_state_msg_.position.push_back(actuator_state.position_interface.state_);
    actuator_state_msg_.velocity.push_back(actuator_state.velocity_interface.state_);
    actuator_state_msg_.effort.push_back(actuator_state.effort_interface.state_);
  }
  // Publish actuator states
  if (actuator_state_realtime_publisher_)
  {
#if ROS_DISTRO_HUMBLE
    actuator_state_realtime_publisher_->tryPublish(actuator_state_msg_);
#else
    actuator_state_realtime_publisher_->try_publish(actuator_state_msg_);
#endif
  }

  actuator_state_to_joint_state();

  // IMU Sensor data
  for (auto& data : imu_sensor_data_)
  {
    data.orientation.data.w() = mj_data_control_->sensordata[data.orientation.mj_sensor_index];
    data.orientation.data.x() = mj_data_control_->sensordata[data.orientation.mj_sensor_index + 1];
    data.orientation.data.y() = mj_data_control_->sensordata[data.orientation.mj_sensor_index + 2];
    data.orientation.data.z() = mj_data_control_->sensordata[data.orientation.mj_sensor_index + 3];

    data.angular_velocity.data.x() = mj_data_control_->sensordata[data.angular_velocity.mj_sensor_index];
    data.angular_velocity.data.y() = mj_data_control_->sensordata[data.angular_velocity.mj_sensor_index + 1];
    data.angular_velocity.data.z() = mj_data_control_->sensordata[data.angular_velocity.mj_sensor_index + 2];

    data.linear_acceleration.data.x() = mj_data_control_->sensordata[data.linear_acceleration.mj_sensor_index];
    data.linear_acceleration.data.y() = mj_data_control_->sensordata[data.linear_acceleration.mj_sensor_index + 1];
    data.linear_acceleration.data.z() = mj_data_control_->sensordata[data.linear_acceleration.mj_sensor_index + 2];
  }

  // FT Sensor data
  for (auto& data : ft_sensor_data_)
  {
    data.force.data.x() = -mj_data_control_->sensordata[data.force.mj_sensor_index];
    data.force.data.y() = -mj_data_control_->sensordata[data.force.mj_sensor_index + 1];
    data.force.data.z() = -mj_data_control_->sensordata[data.force.mj_sensor_index + 2];

    data.torque.data.x() = -mj_data_control_->sensordata[data.torque.mj_sensor_index];
    data.torque.data.y() = -mj_data_control_->sensordata[data.torque.mj_sensor_index + 1];
    data.torque.data.z() = -mj_data_control_->sensordata[data.torque.mj_sensor_index + 2];
  }

  // Publish Odometry
  if (free_joint_id_ != -1 && floating_base_realtime_publisher_)
  {
    floating_base_msg_.header.stamp = time;

    // Position
    floating_base_msg_.pose.pose.position.x = mj_data_control_->qpos[free_joint_qpos_adr_];
    floating_base_msg_.pose.pose.position.y = mj_data_control_->qpos[free_joint_qpos_adr_ + 1];
    floating_base_msg_.pose.pose.position.z = mj_data_control_->qpos[free_joint_qpos_adr_ + 2];

    // Orientation (MuJoCo is w, x, y, z)
    floating_base_msg_.pose.pose.orientation.w = mj_data_control_->qpos[free_joint_qpos_adr_ + 3];
    floating_base_msg_.pose.pose.orientation.x = mj_data_control_->qpos[free_joint_qpos_adr_ + 4];
    floating_base_msg_.pose.pose.orientation.y = mj_data_control_->qpos[free_joint_qpos_adr_ + 5];
    floating_base_msg_.pose.pose.orientation.z = mj_data_control_->qpos[free_joint_qpos_adr_ + 6];

    // Linear Velocity
    floating_base_msg_.twist.twist.linear.x = mj_data_control_->qvel[free_joint_qvel_adr_];
    floating_base_msg_.twist.twist.linear.y = mj_data_control_->qvel[free_joint_qvel_adr_ + 1];
    floating_base_msg_.twist.twist.linear.z = mj_data_control_->qvel[free_joint_qvel_adr_ + 2];

    // Angular Velocity
    floating_base_msg_.twist.twist.angular.x = mj_data_control_->qvel[free_joint_qvel_adr_ + 3];
    floating_base_msg_.twist.twist.angular.y = mj_data_control_->qvel[free_joint_qvel_adr_ + 4];
    floating_base_msg_.twist.twist.angular.z = mj_data_control_->qvel[free_joint_qvel_adr_ + 5];

#if ROS_DISTRO_HUMBLE
    floating_base_realtime_publisher_->tryPublish(floating_base_msg_);
#else
    floating_base_realtime_publisher_->try_publish(floating_base_msg_);
#endif
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& period)
{
  // Update mimic joints
  for (auto& joint : urdf_joint_data_)
  {
    if (joint.is_mimic)
    {
      joint.position_interface.command_ =
          joint.mimic_multiplier * urdf_joint_data_.at(joint.mimicked_joint_index).position_interface.command_;
      joint.velocity_interface.command_ =
          joint.mimic_multiplier * urdf_joint_data_.at(joint.mimicked_joint_index).velocity_interface.command_;
      joint.effort_interface.command_ =
          joint.mimic_multiplier * urdf_joint_data_.at(joint.mimicked_joint_index).effort_interface.command_;
    }
  }

  joint_command_to_actuator_command();

  // portable lambda function to compute pid command using either function name for the correct distro
  auto pid_compute_command = [](auto& pid, const auto& error, const auto& period_t) -> double {
#if ROS_DISTRO_HUMBLE
    return pid->computeCommand(error, period_t);
#else
    return pid->compute_command(error, period_t);
#endif
  };

  // Joint commands
  // TODO: Support command limits. For now those ranges can be limited in the mujoco actuators themselves.
  for (auto& actuator : mujoco_actuator_data_)
  {
    if (actuator.actuator_type == ActuatorType::PASSIVE)
    {
      continue;
    }
    if (actuator.is_position_control_enabled)
    {
      mj_data_control_->ctrl[actuator.mj_actuator_id] = actuator.position_interface.command_;
    }
    else if (actuator.is_position_pid_control_enabled)
    {
      double error = actuator.position_interface.command_ - mj_data_->qpos[actuator.mj_pos_adr];
      mj_data_control_->qfrc_applied[actuator.mj_vel_adr] = pid_compute_command(actuator.pos_pid, error, period);
    }
    else if (actuator.is_velocity_control_enabled)
    {
      mj_data_control_->ctrl[actuator.mj_actuator_id] = actuator.velocity_interface.command_;
    }
    else if (actuator.is_velocity_pid_control_enabled)
    {
      double error = actuator.velocity_interface.command_ - mj_data_->qvel[actuator.mj_vel_adr];
      mj_data_control_->qfrc_applied[actuator.mj_vel_adr] = pid_compute_command(actuator.vel_pid, error, period);
    }
    else if (actuator.is_effort_control_enabled)
    {
      mj_data_control_->ctrl[actuator.mj_actuator_id] = actuator.effort_interface.command_;
    }
  }

  return hardware_interface::return_type::OK;
}

void MujocoSystemInterface::actuator_state_to_joint_state()
{
  // actuator: mujoco -> transmission
  std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                [](auto& actuator_interface) { actuator_interface.copy_state_to_transmission(); });

  // transmission: actuator -> joint
  std::for_each(transmission_instances_.begin(), transmission_instances_.end(),
                [](auto& transmission) { transmission->actuator_to_joint(); });

  // joint: transmission -> state
  std::for_each(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                [](auto& joint_interface) { joint_interface.copy_state_from_transmission(); });

  // If the actuator name and joint name is same (which is the case for non transmission joints), we need to copy
  // the state from actuator to joint here as there is no transmission instance to do that.
  for (auto& joint : urdf_joint_data_)
  {
    std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(), [&](auto& actuator_interface) {
      if (actuator_interface.joint_name == joint.name)
      {
        joint.position_interface.state_ = actuator_interface.position_interface.state_;
        joint.velocity_interface.state_ = actuator_interface.velocity_interface.state_;
        joint.effort_interface.state_ = actuator_interface.effort_interface.state_;
      }
    });
  }
}

void MujocoSystemInterface::joint_command_to_actuator_command()
{
  // Transmissions
  std::for_each(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                [](auto& joint_interface) { joint_interface.copy_command_to_transmission(); });

  // transmission -> actuator
  std::for_each(transmission_instances_.begin(), transmission_instances_.end(),
                [](auto& transmission) { transmission->joint_to_actuator(); });

  // set the commands to the mujoco actuators
  std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                [](auto& actuator_interface) { actuator_interface.copy_command_from_transmission(); });

  // If the actuator name and joint name is same (which is the case for non transmission joints), we need to copy
  // the command from joint to actuator here as there is no transmission instance to do that.
  for (auto& joint : urdf_joint_data_)
  {
    std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(), [&](auto& actuator_interface) {
      if (actuator_interface.joint_name == joint.name && actuator_interface.actuator_type != ActuatorType::PASSIVE)
      {
        actuator_interface.position_interface.command_ = joint.position_interface.command_;
        actuator_interface.velocity_interface.command_ = joint.velocity_interface.command_;
        actuator_interface.effort_interface.command_ = joint.effort_interface.command_;
      }
    });
  }
}

bool MujocoSystemInterface::register_mujoco_actuators()
{
  mujoco_actuator_data_.clear();
  mujoco_actuator_data_.resize(mj_model_->nu);

  // Pull the name of the file to load for starting config, if present. We only override start position if that
  // parameter exists and it is not an empty string
  override_mujoco_actuator_positions_ = false;
  auto it = get_hardware_info().hardware_parameters.find("override_start_position_file");
  if (it != get_hardware_info().hardware_parameters.end())
  {
    override_mujoco_actuator_positions_ = !it->second.empty();
  }

  // If we have that file present, load the initial positions from that file to the appropriate mj_data_ structures
  if (override_mujoco_actuator_positions_)
  {
    std::string override_start_position_file = it->second;
    bool success = set_override_start_positions(override_start_position_file);
    if (!success)
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to load override start positions from %s. Falling back to urdf initial positions.",
                   override_start_position_file.c_str());
      override_mujoco_actuator_positions_ = false;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Loaded initial positions from file %s.", override_start_position_file.c_str());
    }
  }
  else
  {
    RCLCPP_INFO(get_logger(),
                "override_start_position_file not passed. Loading initial positions from ros2_control xacro.");
  }

  for (int i = 0; i < mj_model_->nu; i++)
  {
    RCLCPP_DEBUG(get_logger(), "Registering MuJoCo actuator %d/%d", i + 1, mj_model_->nu);
    MuJoCoActuatorData& actuator_data = mujoco_actuator_data_.at(i);

    // Get the name of the joint/tendon corresponding to the actuator ID
    // Get the transmission type and target ID
    int trn_type = mj_model_->actuator_trntype[i];
    int target_id = mj_model_->actuator_trnid[i * 2];  // The first index of the pair

    // Get the name of the actuator
    const char* act_name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, i);
    if (!act_name)
    {
      act_name = "unnamed";
    }

    if (trn_type == mjTRN_JOINT)
    {
      // <motor name="joint1_motor" joint="actuator1" gear="1"/>
      // this should get actuator1 as actuator_name
      const char* joint_name = mj_id2name(mj_model_, mjOBJ_JOINT, target_id);
      if (joint_name)
      {
        actuator_data.joint_name = std::string(joint_name);
        RCLCPP_INFO(get_logger(), "Registering MuJoCo actuator '%s' for joint '%s'", actuator_data.joint_name.c_str(),
                    joint_name);
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Failed to find joint name for actuator '%s'", act_name);
        return false;
      }
    }
    else if (trn_type == mjTRN_TENDON)
    {
      // For tendon actuators, look up a joint with the same name as the actuator!
      // This is hacky but essentially what happened before the transmissions change, which was also hacky.
      int joint_id = mj_name2id(mj_model_, mjOBJ_JOINT, act_name);
      if (joint_id != -1)
      {
        target_id = joint_id;
        actuator_data.joint_name = std::string(act_name);
        RCLCPP_INFO(get_logger(), "Registering MuJoCo tendon actuator '%s' using joint state", act_name);
      }
      else
      {
        RCLCPP_ERROR(get_logger(),
                     "Tendon actuator '%s' has no matching joint. Tendon actuators must be named the same as a joint "
                     "that they will control.",
                     act_name);
        return false;
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Unsupported transmission type '%d' for actuator '%s'", trn_type, act_name);
      return false;
    }
    actuator_data.mj_actuator_id = i;
    actuator_data.mj_pos_adr = mj_model_->jnt_qposadr[target_id];
    actuator_data.mj_vel_adr = mj_model_->jnt_dofadr[target_id];
    actuator_data.mj_joint_type = mj_model_->jnt_type[target_id];
    actuator_data.actuator_type = getActuatorType(mj_model_, actuator_data.mj_actuator_id);

    // Initialize PID controllers for actuators that have them configured
    const auto initialize_position_pids = [&]() -> bool {
// after humble has an additional argument in the PidROS constructor, and uses a different function to initialize from parameters
#if ROS_DISTRO_HUMBLE
      actuator_data.pos_pid = std::make_shared<control_toolbox::PidROS>(
          mujoco_node_, "pid_gains.position." + actuator_data.joint_name, false);
      actuator_data.pos_pid->initPid();
      const auto gains = actuator_data.pos_pid->getGains();
#else
      actuator_data.pos_pid = std::make_shared<control_toolbox::PidROS>(
          mujoco_node_, "pid_gains.position." + actuator_data.joint_name, "", false);
      actuator_data.pos_pid->initialize_from_ros_parameters();
      const auto gains = actuator_data.pos_pid->get_gains();
#endif
      return std::isfinite(gains.p_gain_) && std::isfinite(gains.i_gain_) && std::isfinite(gains.d_gain_);
    };

    const auto initialize_velocity_pids = [&]() -> bool {
// after humble has an additional argument in the PidROS constructor, and uses a different function to initialize from parameters
#if ROS_DISTRO_HUMBLE
      actuator_data.vel_pid = std::make_shared<control_toolbox::PidROS>(
          mujoco_node_, "pid_gains.velocity." + actuator_data.joint_name, false);
      actuator_data.vel_pid->initPid();
      const auto gains = actuator_data.pos_pid->getGains();
#else
      actuator_data.vel_pid = std::make_shared<control_toolbox::PidROS>(
          mujoco_node_, "pid_gains.velocity." + actuator_data.joint_name, "", false);
      actuator_data.vel_pid->initialize_from_ros_parameters();
      const auto gains = actuator_data.vel_pid->get_gains();
#endif
      return std::isfinite(gains.p_gain_) && std::isfinite(gains.i_gain_) && std::isfinite(gains.d_gain_);
    };

    if (actuator_data.actuator_type == ActuatorType::POSITION)
    {
      actuator_data.is_position_control_enabled = true;
    }
    else if (actuator_data.actuator_type == ActuatorType::VELOCITY)
    {
      actuator_data.has_pos_pid = initialize_position_pids();
      actuator_data.is_velocity_control_enabled = true;
    }
    else if (actuator_data.actuator_type == ActuatorType::MOTOR || actuator_data.actuator_type == ActuatorType::CUSTOM)
    {
      actuator_data.has_pos_pid = initialize_position_pids();
      actuator_data.has_vel_pid = initialize_velocity_pids();
      actuator_data.is_effort_control_enabled = true;
    }
    RCLCPP_DEBUG(get_logger(), "Successfully registered actuator '%s'", act_name);
  }

  // now look out for the mujoco joints that do not have any actuator associated with them
  for (int jnt_id = 0; jnt_id < mj_model_->njnt; jnt_id++)
  {
    const auto actuator_it = std::find_if(mujoco_actuator_data_.cbegin(), mujoco_actuator_data_.cend(),
                                          [&mj_model = mj_model_, jnt_id](const MuJoCoActuatorData& actuator) {
                                            return actuator.mj_pos_adr == mj_model->jnt_qposadr[jnt_id];
                                          });
    if (actuator_it == mujoco_actuator_data_.cend() && mj_model_->jnt_type[jnt_id] != mjJNT_FREE &&
        mj_model_->jnt_type[jnt_id] != mjJNT_BALL)
    {
      // no actuator found for this joint, register a passive actuator
      MuJoCoActuatorData passive_actuator;
      passive_actuator.joint_name = std::string(mj_id2name(mj_model_, mjOBJ_JOINT, jnt_id));
      RCLCPP_INFO(get_logger(), "MuJoCo joint '%s' has no associated actuator. Registering as a passive joint.",
                  passive_actuator.joint_name.c_str());
      passive_actuator.mj_actuator_id = -1;  // indicates no actuator
      passive_actuator.mj_pos_adr = mj_model_->jnt_qposadr[jnt_id];
      passive_actuator.mj_vel_adr = mj_model_->jnt_dofadr[jnt_id];
      passive_actuator.mj_joint_type = mj_model_->jnt_type[jnt_id];
      passive_actuator.actuator_type = ActuatorType::PASSIVE;
      mujoco_actuator_data_.push_back(passive_actuator);
    }
  }

  // Set initial values if they are set in the info, or from override start position file
  if (override_mujoco_actuator_positions_)
  {
    RCLCPP_DEBUG(get_logger(),
                 "Initializing actuator position states from override start position file for %zu actuators.",
                 mujoco_actuator_data_.size());

    for (auto& actuator_data : mujoco_actuator_data_)
    {
      actuator_data.position_interface.state_ = mj_data_->qpos[actuator_data.mj_pos_adr];
      actuator_data.velocity_interface.state_ = mj_data_->qvel[actuator_data.mj_vel_adr];
      // We never set data for effort from an initial conditions file
      actuator_data.effort_interface.state_ = 0.0;

      if (actuator_data.actuator_type != ActuatorType::PASSIVE)
      {
        actuator_data.position_interface.command_ = actuator_data.position_interface.state_;
        actuator_data.velocity_interface.command_ = actuator_data.velocity_interface.state_;
        actuator_data.effort_interface.command_ = actuator_data.effort_interface.state_;
      }
    }
  }
  return true;
}

void MujocoSystemInterface::register_urdf_joints(const hardware_interface::HardwareInfo& hardware_info)
{
  // portable lambda function to get pid gains using either function name for the correct distro
#if !ROS_DISTRO_HUMBLE
  auto get_pid_gains = [](auto& pid) -> control_toolbox::Pid::Gains { return pid->get_gains(); };
#endif

  RCLCPP_INFO(get_logger(), "Registering joints...");
  urdf_joint_data_.resize(hardware_info.joints.size());

  for (size_t joint_index = 0; joint_index < hardware_info.joints.size(); joint_index++)
  {
    auto joint = hardware_info.joints.at(joint_index);
    const std::string actuator_name = get_joint_actuator_name(joint.name, hardware_info, mj_model_);
    const auto actuator_it =
        std::find_if(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                     [&actuator_name, this](const MuJoCoActuatorData& actuator) {
                       return (actuator.actuator_type != ActuatorType::PASSIVE) &&
                              ((mj_id2name(mj_model_, mjOBJ_ACTUATOR, actuator.mj_actuator_id) == actuator_name) ||
                               (actuator.joint_name == actuator_name));
                     });
    const bool actuator_exists = actuator_it != mujoco_actuator_data_.end();
    // This isn't a failure the joint just won't be controllable
    RCLCPP_WARN_EXPRESSION(get_logger(), !actuator_exists,
                           "Failed to find actuator for joint : %s. This joint will be treated as a passive joint.",
                           joint.name.c_str());
    RCLCPP_INFO_EXPRESSION(get_logger(), joint.command_interfaces.empty(), "Joint : %s is a passive joint",
                           joint.name.c_str());
    if (!joint.command_interfaces.empty() && !actuator_exists)
    {
      RCLCPP_WARN(get_logger(),
                  "Joint : %s has command interfaces defined but no matching actuator in the MuJoCo model. This joint "
                  "will be treated as a passive joint and no command interfaces will be exported.",
                  joint.name.c_str());
      joint.command_interfaces.clear();
    }

    // Add to the joint hw information map
    joint_hw_info_.insert(std::make_pair(joint.name, joint));

    // Get the information for the URDF Joint data
    URDFJointData& joint_data = urdf_joint_data_.at(joint_index);
    joint_data.name = joint.name;

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
      joint_data.is_mimic = true;
      joint_data.mimicked_joint_index = std::distance(hardware_info.joints.begin(), mimicked_joint_it);

      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        joint_data.mimic_multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      else
      {
        joint_data.mimic_multiplier = 1.0;
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

    // Set initial values to joint interfaces if they are set in the info
    if (!override_mujoco_actuator_positions_)
    {
      override_urdf_joint_positions_ = true;
      for (const auto& state_if : joint.state_interfaces)
      {
        if (state_if.name == hardware_interface::HW_IF_POSITION)
        {
          joint_data.position_interface.state_ = get_initial_value(state_if);
        }
        else if (state_if.name == hardware_interface::HW_IF_VELOCITY)
        {
          joint_data.velocity_interface.state_ = get_initial_value(state_if);
        }
        else if (state_if.name == hardware_interface::HW_IF_EFFORT ||
                 state_if.name == hardware_interface::HW_IF_TORQUE || state_if.name == hardware_interface::HW_IF_FORCE)
        {
          // We never set data for effort from an initial conditions file, so just default to the initial value if it exists.
          joint_data.effort_interface.state_ = get_initial_value(state_if);
        }

        // Copy the initial state also to the command interfaces
        joint_data.position_interface.command_ = joint_data.position_interface.state_;
        joint_data.velocity_interface.command_ = joint_data.velocity_interface.state_;
        joint_data.effort_interface.command_ = joint_data.effort_interface.state_;
      }
    }

    // Command interfaces
    // sort the interfaces so that the order is as follows Position, Velocity, Effort, Anything else
    std::vector<std::string> joint_cmd_itfs = {};
    std::transform(joint.command_interfaces.begin(), joint.command_interfaces.end(), std::back_inserter(joint_cmd_itfs),
                   [](const hardware_interface::InterfaceInfo& interface_info) { return interface_info.name; });
    const auto command_interface_names =
        get_interfaces_in_order(joint_cmd_itfs, { hardware_interface::HW_IF_POSITION,
                                                  hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT,
                                                  hardware_interface::HW_IF_TORQUE, hardware_interface::HW_IF_FORCE });
    joint_data.command_interfaces = command_interface_names;

    for (const auto& command_if : command_interface_names)
    {
      // If available, always default to position control at the start
      if (command_if == hardware_interface::HW_IF_POSITION)
      {
        // Position command interface
        // Direct control for position actuators; position PID required for velocity, motor, or custom actuators.

        if (actuator_it->actuator_type == ActuatorType::POSITION)
        {
          RCLCPP_INFO(get_logger(), "Using MuJoCo position actuator for the joint : '%s'", actuator_name.c_str());
          // Direct position control enabled for position actuator
          actuator_it->is_position_control_enabled = true;
        }
        else if (actuator_it->actuator_type == ActuatorType::VELOCITY ||
                 actuator_it->actuator_type == ActuatorType::MOTOR ||
                 actuator_it->actuator_type == ActuatorType::CUSTOM)
        {
          if (actuator_it->has_pos_pid)
          {
            actuator_it->is_position_control_enabled = false;
            actuator_it->is_position_pid_control_enabled = true;

// just disabling for humble because the member variables are different. Could make a different one for humble if desired
#if !ROS_DISTRO_HUMBLE
            const auto gains = get_pid_gains(actuator_it->pos_pid);
            RCLCPP_INFO(get_logger(),
                        "Position control PID gains for joint %s : P=%.4f, I=%.4f, D=%.4f, Imax=%.4f, Imin=%.4f, "
                        "Umin=%.4f, Umax=%.4f, antiwindup_strategy=%s",
                        actuator_name.c_str(), gains.p_gain_, gains.i_gain_, gains.d_gain_,
                        gains.antiwindup_strat_.i_max, gains.antiwindup_strat_.i_min, gains.u_min_, gains.u_max_,
                        gains.antiwindup_strat_.to_string().c_str());
#endif
          }
          else
          {
            RCLCPP_ERROR(get_logger(),
                         "Position command interface for the joint : %s is not supported with velocity or motor "
                         "actuator without defining the PIDs",
                         actuator_name.c_str());
          }
        }
      }
      else if (command_if == hardware_interface::HW_IF_VELOCITY)
      {
        // Velocity command interface:
        // Direct control for velocity actuators; velocity PID required for motor or custom actuators.
        RCLCPP_ERROR_EXPRESSION(get_logger(), actuator_it->actuator_type == ActuatorType::POSITION,
                                "Velocity command interface for the joint : %s is not supported with position actuator",
                                actuator_name.c_str());
        if (actuator_it->actuator_type == ActuatorType::VELOCITY)
        {
          RCLCPP_INFO(get_logger(), "Using MuJoCo velocity actuator for the joint : '%s'", actuator_name.c_str());
          // Direct velocity control enabled for velocity actuator
          actuator_it->is_velocity_control_enabled = true;
        }
        else if (actuator_it->actuator_type == ActuatorType::MOTOR || actuator_it->actuator_type == ActuatorType::CUSTOM)
        {
          if (actuator_it->has_vel_pid)
          {
            actuator_it->is_velocity_control_enabled = false;
            actuator_it->is_velocity_pid_control_enabled = true;
// just disabling for humble because the member variables are different. Could make a different one for humble if desired
#if !ROS_DISTRO_HUMBLE
            const auto gains = get_pid_gains(actuator_it->vel_pid);
            RCLCPP_INFO(get_logger(),
                        "Velocity control PID gains for joint %s : P=%.4f, I=%.4f, D=%.4f, Imax=%.4f, Imin=%.4f, "
                        "Umin=%.4f, Umax=%.4f, antiwindup_strategy=%s",
                        actuator_name.c_str(), gains.p_gain_, gains.i_gain_, gains.d_gain_,
                        gains.antiwindup_strat_.i_max, gains.antiwindup_strat_.i_min, gains.u_min_, gains.u_max_,
                        gains.antiwindup_strat_.to_string().c_str());
#endif
          }
          else
          {
            RCLCPP_ERROR(get_logger(),
                         "Velocity command interface for the joint : %s is not supported with motor or custom actuator "
                         "without defining the PIDs",
                         actuator_name.c_str());
          }
        }
      }
      else if (command_if == hardware_interface::HW_IF_EFFORT || command_if == hardware_interface::HW_IF_TORQUE ||
               command_if == hardware_interface::HW_IF_FORCE)
      {
        // Effort command interface:
        // Direct control for effort actuators; not supported for position or velocity actuators.
        RCLCPP_ERROR_EXPRESSION(
            get_logger(),
            actuator_it->actuator_type == ActuatorType::POSITION || actuator_it->actuator_type == ActuatorType::VELOCITY,
            "Effort command interface for the joint : %s is not supported with position or velocity actuator."
            "Skipping it.",
            actuator_name.c_str());
        if (actuator_it->actuator_type == ActuatorType::MOTOR || actuator_it->actuator_type == ActuatorType::CUSTOM)
        {
          RCLCPP_INFO(get_logger(), "Using MuJoCo motor or custom actuator for the joint : '%s'", actuator_name.c_str());
          // Direct effort control enabled for MOTOR or CUSTOM actuator
          actuator_it->is_effort_control_enabled = true;
        }
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Unsupported command interface '%s' for joint '%s'. Skipping it!", command_if.c_str(),
                    joint.name.c_str());
      }
    }
    if (!command_interface_names.empty() && !actuator_it->is_position_control_enabled &&
        !actuator_it->is_velocity_control_enabled && !actuator_it->is_effort_control_enabled &&
        !actuator_it->is_position_pid_control_enabled && !actuator_it->is_velocity_pid_control_enabled)
    {
      throw std::runtime_error("Joint '" + joint.name + "' which uses actuator '" + actuator_name +
                               "' has an unsupported command interface for the specified MuJoCo actuator");
    }
  }
}

bool MujocoSystemInterface::register_transmissions(const hardware_interface::HardwareInfo& hardware_info)
{
  transmission_instances_.clear();
  auto hardware_transmissions = hardware_info.transmissions;
  transmission_loader_ = std::make_unique<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
      "transmission_interface", "transmission_interface::TransmissionLoader");

  for (const auto& t_info : hardware_transmissions)
  {
    if (t_info.joints.empty() || t_info.actuators.empty())
    {
      RCLCPP_FATAL(get_logger(), "Transmission '%s' has no joints or actuators defined", t_info.name.c_str());
      return false;
    }

    // If the joints are not found as mujoco actuators, check the actuators of transmission
    bool all_transmission_actuators = true;
    for (const auto& tran_actuator_info : t_info.actuators)
    {
      RCLCPP_DEBUG(get_logger(), "Actuator name: %s", tran_actuator_info.name.c_str());

      if (get_actuator_id(tran_actuator_info.name, mj_model_) != -1)
      {
        RCLCPP_INFO(get_logger(), "Transmission actuator '%s' matches the MuJoCo actuator",
                    tran_actuator_info.name.c_str());
        all_transmission_actuators &= true;
      }
      else
      {
        RCLCPP_WARN(get_logger(), "Transmission actuator '%s' not found in MuJoCo model",
                    tran_actuator_info.name.c_str());
        all_transmission_actuators &= false;
      }
    }

    if (!all_transmission_actuators)
    {
      RCLCPP_ERROR(get_logger(),
                   "Not all transmission actuators and joints for transmission '%s' found as MuJoCo actuators. This "
                   "shouldn't happen.",
                   t_info.name.c_str());
      return false;
    }

    if (!transmission_loader_->isClassAvailable(t_info.type))
    {
      RCLCPP_FATAL(get_logger(), "Transmission '%s' of type '%s' not available", t_info.name.c_str(),
                   t_info.type.c_str());
      return false;
    }

    // command interfaces of the all joints of the same transmission must be the same (order is not important)
    const auto first_joint_command_interfaces = t_info.joints.front().command_interfaces;
    for (const auto& joint_info : t_info.joints)
    {
      const auto joint_hw_types = joint_info.command_interfaces;
      bool is_same = (joint_hw_types.size() == first_joint_command_interfaces.size()) &&
                     std::all_of(joint_hw_types.begin(), joint_hw_types.end(), [&](const std::string& hw_type) {
                       return std::find(first_joint_command_interfaces.begin(), first_joint_command_interfaces.end(),
                                        hw_type) != first_joint_command_interfaces.end();
                     });
      if (!is_same)
      {
        RCLCPP_FATAL(get_logger(),
                     "All joints of transmission '%s' must have the same command interfaces. Joint '%s' has different "
                     "interfaces than joint '%s'.",
                     t_info.name.c_str(), joint_info.name.c_str(), t_info.joints.front().name.c_str());
        return false;
      }
    }

    std::shared_ptr<transmission_interface::Transmission> transmission = nullptr;
    try
    {
      auto loader = transmission_loader_->createSharedInstance(t_info.type);
      transmission = loader->load(t_info);
    }
    catch (const std::exception& e)
    {
      RCLCPP_FATAL(get_logger(), "Caught exception when trying to create transmission loader of type %s : %s",
                   t_info.type.c_str(), e.what());
      return false;
    }

    // Create the joint_handles vector for each joint in the transmission
    std::vector<transmission_interface::JointHandle> joint_handles;
    RCLCPP_INFO(get_logger(), "Creating joint and actuator handles for transmission: %s", t_info.name.c_str());
    for (const auto& joint_info : t_info.joints)
    {
      RCLCPP_INFO(get_logger(), "\tCreating joint handle for joint: %s", joint_info.name.c_str());

      std::vector<std::string> joint_hw_types = joint_info.state_interfaces;
      mujoco_ros2_control::add_items(joint_hw_types, joint_info.command_interfaces);

      // Get the URDFJointData of the joint to set the control flags
      auto urdf_joint_it = std::find_if(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                                        [&](const auto& js) { return js.name == joint_info.name; });
      if (urdf_joint_it == urdf_joint_data_.end())
      {
        RCLCPP_FATAL(get_logger(), "Joint '%s' not found in the URDF joint data", joint_info.name.c_str());
        return false;
      }

      for (const auto& hw_if : joint_hw_types)
      {
        double* passthrough = nullptr;
        if (hw_if == hardware_interface::HW_IF_POSITION)
        {
          passthrough = &(urdf_joint_it->position_interface.transmission_passthrough_);
        }
        else if (hw_if == hardware_interface::HW_IF_VELOCITY)
        {
          passthrough = &(urdf_joint_it->velocity_interface.transmission_passthrough_);
        }
        else if (hw_if == hardware_interface::HW_IF_EFFORT || hw_if == hardware_interface::HW_IF_TORQUE ||
                 hw_if == hardware_interface::HW_IF_FORCE)
        {
          passthrough = &(urdf_joint_it->effort_interface.transmission_passthrough_);
        }
        transmission_interface::JointHandle joint_handle(joint_info.name, hw_if, passthrough);
        joint_handles.push_back(joint_handle);
      }
    }

    // Create the actuator_handles vector for each actuator in the transmission
    std::vector<transmission_interface::ActuatorHandle> actuator_handles;
    for (const auto& tran_actuator_info : t_info.actuators)
    {
      RCLCPP_INFO(get_logger(), "\tCreating actuator handle for actuator: %s", tran_actuator_info.name.c_str());
      const std::vector<std::string> hw_types = { hardware_interface::HW_IF_POSITION,
                                                  hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT,
                                                  hardware_interface::HW_IF_TORQUE, hardware_interface::HW_IF_FORCE };

      auto mujoco_actuator_it = std::find_if(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                                             [&](const auto& ma) { return ma.joint_name == tran_actuator_info.name; });
      if (mujoco_actuator_it == mujoco_actuator_data_.end())
      {
        RCLCPP_FATAL(get_logger(), "Actuator '%s' not found in the MuJoCo actuator data",
                     tran_actuator_info.name.c_str());
        return false;
      }
      if (mujoco_actuator_it->actuator_type == ActuatorType::PASSIVE)
      {
        RCLCPP_FATAL(get_logger(), "Actuator '%s' is passive and cannot be used in a transmission",
                     tran_actuator_info.name.c_str());
        return false;
      }
      for (const auto& hw_if : hw_types)
      {
        double* passthrough = nullptr;
        if (hw_if == hardware_interface::HW_IF_POSITION)
        {
          passthrough = &(mujoco_actuator_it->position_interface.transmission_passthrough_);
        }
        else if (hw_if == hardware_interface::HW_IF_VELOCITY)
        {
          passthrough = &(mujoco_actuator_it->velocity_interface.transmission_passthrough_);
        }
        else if (hw_if == hardware_interface::HW_IF_EFFORT || hw_if == hardware_interface::HW_IF_TORQUE ||
                 hw_if == hardware_interface::HW_IF_FORCE)
        {
          passthrough = &(mujoco_actuator_it->effort_interface.transmission_passthrough_);
        }
        transmission_interface::ActuatorHandle actuator_handle(tran_actuator_info.name, hw_if, passthrough);
        actuator_handles.push_back(actuator_handle);
      }
    }

    try
    {
      transmission->configure(joint_handles, actuator_handles);
    }
    catch (const transmission_interface::TransmissionInterfaceException& exc)
    {
      RCLCPP_FATAL(get_logger(), "Error while configuring %s: %s", t_info.name.c_str(), exc.what());
      return false;
    }

    transmission_instances_.push_back(transmission);
  }
  RCLCPP_INFO_EXPRESSION(get_logger(), !transmission_instances_.empty(), "Registered %zu transmissions",
                         transmission_instances_.size());

  return true;
}

bool MujocoSystemInterface::initialize_initial_positions(const hardware_interface::HardwareInfo& /*hardware_info*/)
{
  if (override_mujoco_actuator_positions_)
  {
    // Transforms the actuators' state to the joint state interfaces
    actuator_state_to_joint_state();

    // Set the initial joint state as joint commands
    std::for_each(urdf_joint_data_.begin(), urdf_joint_data_.end(),
                  [](auto& joint_interface) { joint_interface.copy_state_to_command(); });
  }
  if (override_urdf_joint_positions_)
  {
    // Transforms the joints' command to the actuator command interfaces
    joint_command_to_actuator_command();

    // Set the initial actuator commands as actuator states
    std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(),
                  [](auto& actuator_interface) { actuator_interface.copy_command_to_state(); });

    // Copy the initial joint state to the actuator state for the passive joints
    // If the actuator name and joint name is same (which is the case for non transmission joints), we need to copy
    // the state from actuator to joint here as there is no transmission instance to do that.
    for (auto& joint : urdf_joint_data_)
    {
      std::for_each(mujoco_actuator_data_.begin(), mujoco_actuator_data_.end(), [&](auto& actuator_interface) {
        if (actuator_interface.joint_name == joint.name && actuator_interface.actuator_type == ActuatorType::PASSIVE)
        {
          actuator_interface.position_interface.state_ = joint.position_interface.state_;
          actuator_interface.velocity_interface.state_ = joint.velocity_interface.state_;
          actuator_interface.effort_interface.state_ = joint.effort_interface.state_;
        }
      });
    }
  }
  return true;
}

void MujocoSystemInterface::register_sensors(const hardware_interface::HardwareInfo& hardware_info)
{
  for (size_t sensor_index = 0; sensor_index < hardware_info.sensors.size(); sensor_index++)
  {
    auto sensor = hardware_info.sensors.at(sensor_index);
    const std::string sensor_name = sensor.name;

    if (sensor.parameters.count("mujoco_type") == 0)
    {
      RCLCPP_INFO_STREAM(get_logger(),
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

    RCLCPP_INFO_STREAM(get_logger(), "Adding sensor named: " << sensor_name << ", of type: " << mujoco_type
                                                             << ", mapping to the MJCF sensor: " << mujoco_sensor_name);

    // Add to the sensor hw information map
    sensors_hw_info_.insert(std::make_pair(sensor_name, sensor));

    if (mujoco_type == "fts")
    {
      FTSensorData sensor_data;
      sensor_data.name = sensor_name;
      sensor_data.force.name =
          mujoco_sensor_name + get_hardware_parameter_or(get_hardware_info(), "force_mjcf_suffix", "_force");
      sensor_data.torque.name =
          mujoco_sensor_name + get_hardware_parameter_or(get_hardware_info(), "torque_mjcf_suffix", "_torque");

      int force_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.force.name.c_str());
      int torque_sensor_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.torque.name.c_str());

      if (force_sensor_id == -1 || torque_sensor_id == -1)
      {
        RCLCPP_ERROR_STREAM(get_logger(),
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
      sensor_data.orientation.name =
          mujoco_sensor_name + get_hardware_parameter_or(get_hardware_info(), "orientation_mjcf_suffix", "_quat");
      sensor_data.angular_velocity.name =
          mujoco_sensor_name + get_hardware_parameter_or(get_hardware_info(), "angular_velocity_mjcf_suffix", "_gyro");
      sensor_data.linear_acceleration.name =
          mujoco_sensor_name +
          get_hardware_parameter_or(get_hardware_info(), "linear_acceleration_mjcf_suffix", "_accel");

      // Initialize to all zeros as we do not use these yet.
      sensor_data.orientation_covariance.resize(9, 0.0);
      sensor_data.angular_velocity_covariance.resize(9, 0.0);
      sensor_data.linear_acceleration_covariance.resize(9, 0.0);

      int quat_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.orientation.name.c_str());
      int gyro_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.angular_velocity.name.c_str());
      int accel_id = mj_name2id(mj_model_, mjOBJ_SENSOR, sensor_data.linear_acceleration.name.c_str());

      if (quat_id == -1 || gyro_id == -1 || accel_id == -1)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Failed to find IMU sensor in mujoco model, sensor name: " << sensor.name);
        continue;
      }

      sensor_data.orientation.mj_sensor_index = mj_model_->sensor_adr[quat_id];
      sensor_data.angular_velocity.mj_sensor_index = mj_model_->sensor_adr[gyro_id];
      sensor_data.linear_acceleration.mj_sensor_index = mj_model_->sensor_adr[accel_id];

      imu_sensor_data_.push_back(sensor_data);
    }
    else
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid mujoco_type passed to the mujoco hardware interface: " << mujoco_type);
    }
  }
}

bool MujocoSystemInterface::set_override_start_positions(const std::string& override_start_position_file)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(override_start_position_file.c_str()) != tinyxml2::XML_SUCCESS)
  {
    RCLCPP_ERROR_STREAM(get_logger(),
                        "Failed to load override start position file " << override_start_position_file.c_str() << ".");
    return false;
  }

  // get the <key> element
  tinyxml2::XMLElement* keyElem = doc.FirstChildElement("key");
  if (!keyElem)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "<key> element not found in override start position file.");
    return false;
  }

  auto parseAttr = [&](tinyxml2::XMLElement* elem, const char* attrName) -> std::vector<double> {
    std::vector<double> result;

    const char* text = elem->Attribute(attrName);
    if (!text)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Attribute '" << attrName << "' not found in override start position file.");
      return result;  // return empty vector
    }

    std::stringstream ss(text);
    double v;
    while (ss >> v)
    {
      result.push_back(v);
    }

    return result;
  };

  std::vector<double> qpos = parseAttr(keyElem, "qpos");
  std::vector<double> qvel = parseAttr(keyElem, "qvel");
  std::vector<double> ctrl = parseAttr(keyElem, "ctrl");

  // we already put out an error message saying that it couldn't load specific things, so we don't need to say anything else
  if (qpos.empty() || qvel.empty() || ctrl.empty())
  {
    return false;
  }

  if ((qpos.size() != static_cast<size_t>(mj_model_->nq)) || (qvel.size() != static_cast<size_t>(mj_model_->nv)) ||
      (ctrl.size() != static_cast<size_t>(mj_model_->nu)))
  {
    RCLCPP_ERROR_STREAM(
        get_logger(), "Mismatch in data types in override starting positions. Numbers are:\n\t"
                          << "qpos size in file: " << qpos.size() << ", qpos size in model: " << mj_model_->nq << "\n\t"
                          << "qvel size in file: " << qvel.size() << ", qvel size in model: " << mj_model_->nv << "\n\t"
                          << "ctrl size in file: " << ctrl.size() << ", ctrl size in model: " << mj_model_->nu);
    return false;
  }

  // copy data from the input information into the mj_data_ object
  std::copy(qpos.begin(), qpos.end(), mj_data_->qpos);
  std::copy(qvel.begin(), qvel.end(), mj_data_->qvel);
  std::copy(ctrl.begin(), ctrl.end(), mj_data_->ctrl);

  return true;
}

void MujocoSystemInterface::set_initial_pose()
{
  for (auto& actuator : mujoco_actuator_data_)
  {
    // Check if the actuator data is finite before setting it. This check is needed if the MuJoCo model has more passive
    // joints, than those exported/used in ros2_control, then the state_ variables will be left as NaN. So, better to
    // leave it to the MuJoCo model's default initial position.
    if (std::isfinite(actuator.position_interface.state_))
    {
      mj_data_->qpos[actuator.mj_pos_adr] = actuator.position_interface.state_;
    }
    else
    {
      RCLCPP_WARN_EXPRESSION(
          get_logger(), actuator.actuator_type != ActuatorType::PASSIVE,
          "Actuator '%s' position state is not finite. Leaving it to the MuJoCo model's default initial position.",
          actuator.joint_name.c_str());
    }
    if (actuator.is_position_control_enabled)
    {
      mj_data_->ctrl[actuator.mj_actuator_id] = actuator.position_interface.state_;
    }
    else if (actuator.is_velocity_control_enabled)
    {
      mj_data_->ctrl[actuator.mj_actuator_id] = actuator.velocity_interface.state_;
    }
    else if (actuator.is_effort_control_enabled)
    {
      mj_data_->ctrl[actuator.mj_actuator_id] = actuator.effort_interface.state_;
    }
  }

  // Copy into the control data for reads
  mj_copyData(mj_data_control_, mj_model_, mj_data_);
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
    //  yield results in busy wait - which has better timing but kills battery life
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
          auto elapsedSim = mj_data_->time - syncSim;

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

            // Copy data to the control
            mju_copy(mj_data_->ctrl, mj_data_control_->ctrl, mj_model_->nu);
            mju_copy(mj_data_->qfrc_applied, mj_data_control_->qfrc_applied, mj_model_->nu);
            // run single step, let next iteration deal with timing
            mj_step(mj_model_, mj_data_);

            // Publish clock after each successful step
            publish_clock();

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
            auto currentCPU = mj::Simulate::Clock::now();
            while (Seconds((mj_data_->time - syncSim) * speedFactor) < currentCPU - syncCPU &&
                   currentCPU - startCPU < Seconds(refreshTime))
            {
              // measure slowdown before first step
              if (!measured && elapsedSim)
              {
                sim_->measured_slowdown =
                    static_cast<float>(std::chrono::duration<double>(elapsedCPU).count() / elapsedSim);
                measured = true;
              }
// inject noise
// Use mjVERSION_HEADER and if it is greater than 337 then do one thing or another
// Needed due to
// https://github.com/google-deepmind/mujoco/commit/401bf431b8b0fe6e0a619412a607b5135dc4ded4#diff-3dc22ceeebd71304c41d349c6d273bda172ea88ff49c772dbdcf51b9b19bbd33R2943
#if mjVERSION_HEADER < 337
              sim_->InjectNoise();
#else
              sim_->InjectNoise(-1);
#endif

              // Copy data to the control
              mju_copy(mj_data_->ctrl, mj_data_control_->ctrl, mj_model_->nu);
              mju_copy(mj_data_->qfrc_applied, mj_data_control_->qfrc_applied, mj_model_->nu);
              // call mj_step
              mj_step(mj_model_, mj_data_);

              // Publish clock after each successful step
              publish_clock();

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

              // Update current CPU time for next iteration
              currentCPU = mj::Simulate::Clock::now();
            }
          }

          // save current state to history buffer
          if (stepped)
          {
            // Update the control's read buffers if the data has changed
            mj_copyData(mj_data_control_, mj_model_, mj_data_);

            sim_->AddToHistory();
          }
        }

        // paused
        else
        {
          mj_copyData(mj_data_control_, mj_model_, mj_data_);

          // run mj_forward, to update rendering and joint sliders
          mj_forward(mj_model_, mj_data_);
          sim_->speed_changed = true;
        }
      }
    }  // release std::lock_guard<std::mutex>
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
// fixing for different naming convention on humble vs everything else
#if ROS_DISTRO_HUMBLE
  clock_realtime_publisher_->tryPublish(sim_time_msg);
#else
  clock_realtime_publisher_->try_publish(sim_time_msg);
#endif
}

void MujocoSystemInterface::get_model(mjModel*& dest)
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  dest = mj_copyModel(dest, mj_model_);
}

void MujocoSystemInterface::get_data(mjData*& dest)
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  if (dest == nullptr)
  {
    dest = mj_makeData(mj_model_);
  }
  mj_copyData(dest, mj_model_, mj_data_);
}

void MujocoSystemInterface::set_data(mjData* mj_data)
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  mj_copyData(mj_data_, mj_model_, mj_data);
}

rclcpp::Logger MujocoSystemInterface::get_logger() const
{
  return logger_;
}

}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystemInterface, hardware_interface::SystemInterface);
