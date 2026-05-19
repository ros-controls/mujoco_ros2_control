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

#include "mujoco_ros2_control/mujoco_simulation.hpp"
#include "array_safety.h"

#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>

#include <std_msgs/msg/string.hpp>

#include <hardware_interface/version.h>
#include <rclcpp/version.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include "lodepng.h"

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

using namespace std::chrono_literals;

// constants
const double kSyncMisalign = 0.1;        // maximum misalignment before re-sync (simulation seconds)
const double kSimRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;           // load error string length

using Seconds = std::chrono::duration<double>;

namespace mujoco_ros2_control
{
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

/**
 * GlfwAdapter subclass that overrides right-arrow-key handling so that a single
 * simulation step is driven exclusively by the ROS control loop rather than by
 * MuJoCo's built-in viewer.
 *
 * When the simulation is paused, MuJoCo's default GlfwAdapter advances the
 * physics by one step (mj_step) on each right-arrow press or key-repeat event.
 * This class suppresses that native behaviour and instead sets step_requested_,
 * which the ROS control loop polls to advance the simulation.  This ensures
 * that ros2_controller read/update/write cycles are executed for every step and
 * that controller state remains consistent with the physics.
 *
 * All other keys are forwarded to the parent class unchanged so that the rest
 * of the MuJoCo viewer UI (play/pause, reset, rendering options, etc.) works
 * as normal.
 */
class ROS2ControlGlfwAdapter : public mj::GlfwAdapter
{
public:
  explicit ROS2ControlGlfwAdapter(std::atomic<bool>& step_requested) : step_requested_(step_requested)
  {
  }

protected:
  void OnKey(int key, int scancode, int act) override
  {
    // Intercept the right arrow key so only the ROS loop advances the physics,
    // preventing double-stepping (MuJoCo's native handler would also call mj_step).
    if (key == GLFW_KEY_RIGHT)
    {
      if (act == GLFW_PRESS || act == GLFW_REPEAT)
      {
        step_requested_.store(true);
      }
      return;
    }

    // Forward all other keys so normal UI behaviour is preserved.
    mj::GlfwAdapter::OnKey(key, scancode, act);
  }

private:
  std::atomic<bool>& step_requested_;
};

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
static std::string getExecutableDir()
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
static void scanPluginLibraries()
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

static const char* Diverged(int disableflags, const mjData* d)
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

static mjModel* loadModelFromFile(const char* file, mj::Simulate& sim)
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

static mjModel* loadModelFromTopic(rclcpp::Node::SharedPtr node, const std::string& topic_name)
{
  mjModel* mnew = 0;
  std::string robot_description;

  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.reliable().transient_local().keep_last(1);
  RCLCPP_INFO(node->get_logger(), "Trying to get the MuJoCo model from topic '%s'", topic_name.c_str());

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
      RCLCPP_WARN(node->get_logger(), "Timeout waiting for '%s' topic.", topic_name.c_str());
      return nullptr;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  if (!robot_description.empty())
  {
    // Load MuJoCo model
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
      return nullptr;
    }
    mj_deleteSpec(spec);
    RCLCPP_INFO(node->get_logger(), "Model body count: %ld", static_cast<long>(mnew->nbody));
    RCLCPP_INFO(node->get_logger(), "Model geom count: %ld", static_cast<long>(mnew->ngeom));
  }
  return mnew;
}

static mjModel* LoadModel(const char* file, const std::string& topic, mj::Simulate& sim, rclcpp::Node::SharedPtr node)
{
  // Try to get the MuJoCo model from URDF.
  // If it is not available, create a subscription and listen for the model on a topic.

  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // load model from path if the filename is not empty
  if (filename[0])
  {
    return loadModelFromFile(file, sim);
  }
  // Try to get the MuJoCo model from topic
  return loadModelFromTopic(node, topic);
}

MujocoSimulation::~MujocoSimulation()
{
  shutdown();

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

bool MujocoSimulation::initialize(rclcpp::Node::SharedPtr node, const std::string& model_path,
                                  const std::string& mujoco_model_topic, double sim_speed_factor, bool headless)
{
  node_ = node;
  model_path_ = model_path;
  mujoco_model_topic_ = mujoco_model_topic;
  sim_speed_factor_ = sim_speed_factor;
  headless_ = headless;

  if (sim_speed_factor_ > 0)
  {
    RCLCPP_INFO(get_logger(), "Running the simulation at %.2f percent speed", sim_speed_factor_ * 100.0);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "No sim_speed set, using the setting from the UI");
  }

  RCLCPP_INFO_EXPRESSION(get_logger(), headless_, "Running in HEADLESS mode.");

  // We essentially reconstruct the 'simulate.cc::main()' function here, and
  // launch a Simulate object with all necessary rendering process/options
  // attached.

  // scan for libraries in the plugin directory to load additional plugins
  RCLCPP_INFO(get_logger(), "Scanning plugin libraries...");
  scanPluginLibraries();

  // Retain scope
  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultPerturb(&pert_);

  // There is a timing issue here as the rendering context must be attached to
  // the executing thread, but we require the simulation to be available on
  // init. So we spawn the sim in the rendering thread prior to proceeding with
  // initialization.
  RCLCPP_INFO(get_logger(), "Initializing simulation...");
  auto sim_ready = std::make_shared<std::promise<void>>();
  std::future<void> sim_ready_future = sim_ready->get_future();

  if (headless_)
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
      sim_ = std::make_unique<mj::Simulate>(std::make_unique<ROS2ControlGlfwAdapter>(keyboard_step_requested_), &cam_,
                                            &opt_, &pert_,
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
        RCLCPP_WARN(get_logger(), "LodePNG error %u: %s. Icon file not loaded: %s", error, lodepng_error_text(error),
                    icon_location.c_str());
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
      RCLCPP_INFO(get_logger(), "Starting the MuJoCo rendering thread...");
      sim_->RenderLoop();
    });
  }

  if (sim_ready_future.wait_for(10s) == std::future_status::timeout)
  {
    RCLCPP_FATAL(get_logger(), "Timed out waiting to start simulation rendering!");
    return false;
  }
  RCLCPP_INFO(get_logger(), "Sim ready, continuing initialization...");

  // We maintain a pointer to the mutex so that we can lock from here, too.
  // Is this a terrible idea? Maybe, but it lets us use their libraries as is...
  sim_mutex_ = &sim_->mtx;

  // Load the model and data prior to hw registration and starting the physics thread
  sim_->LoadMessage(model_path_.c_str());

  // Time publisher will be pushed from the physics_thread_
  RCLCPP_INFO(get_logger(), "Constructing clock publisher.");
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  clock_realtime_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<rosgraph_msgs::msg::Clock>>(clock_publisher_);

  // Initialize services
  // For humble compatibility.
#if RCLCPP_VERSION_MAJOR >= 17
  rclcpp::QoS qos_services =
      rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1)).reliable().durability_volatile();
#else
  const rmw_qos_profile_t qos_services = { RMW_QOS_POLICY_HISTORY_KEEP_ALL,
                                           1,  // message queue depth
                                           RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                           RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                           RMW_QOS_DEADLINE_DEFAULT,
                                           RMW_QOS_LIFESPAN_DEFAULT,
                                           RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                           RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                           false };
#endif
  reset_world_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  set_pause_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  step_simulation_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  reset_world_service_ = node_->create_service<mujoco_ros2_control_msgs::srv::ResetWorld>(
      "~/reset_world",
      std::bind(&MujocoSimulation::reset_world_callback, this, std::placeholders::_1, std::placeholders::_2),
      qos_services, reset_world_cb_group_);
  RCLCPP_INFO(get_logger(), "Created reset_world service at: %s/reset_world", node_->get_fully_qualified_name());

  set_pause_service_ = node_->create_service<mujoco_ros2_control_msgs::srv::SetPause>(
      "~/set_pause",
      std::bind(&MujocoSimulation::set_pause_callback, this, std::placeholders::_1, std::placeholders::_2),
      qos_services, set_pause_cb_group_);
  RCLCPP_INFO(get_logger(), "Created set_pause service at: %s/set_pause", node_->get_fully_qualified_name());

  step_simulation_service_ = node_->create_service<mujoco_ros2_control_msgs::srv::StepSimulation>(
      "~/step_simulation",
      std::bind(&MujocoSimulation::step_simulation_callback, this, std::placeholders::_1, std::placeholders::_2),
      qos_services, step_simulation_cb_group_);
  RCLCPP_INFO(get_logger(), "Created step_simulation service at: %s/step_simulation", node_->get_fully_qualified_name());

  // Finish initialization by loading the model and initializing the model and control data containers.
  RCLCPP_INFO(get_logger(), "Loading model...");
  mj_model_ = LoadModel(model_path_.c_str(), mujoco_model_topic_, *sim_, node_);
  if (!mj_model_)
  {
    RCLCPP_FATAL(get_logger(), "MuJoCo failed to load the model");
    return false;
  }

  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    mj_data_ = mj_makeData(mj_model_);
    mj_data_control_ = mj_makeData(mj_model_);
  }
  if (!mj_data_ || !mj_data_control_)
  {
    RCLCPP_FATAL(get_logger(), "Could not allocate mjData for '%s'", model_path_.c_str());
    return false;
  }

  return true;
}

bool MujocoSimulation::apply_keyframe(const std::string& keyframe_name)
{
  int keyframe_id = mj_name2id(mj_model_, mjOBJ_KEY, keyframe_name.c_str());
  if (keyframe_id == -1)
  {
    RCLCPP_ERROR(get_logger(), "Failed to find keyframe : '%s' in the MuJoCo model!", keyframe_name.c_str());
    return false;
  }

  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);

  const auto prev_sim_time = mj_data_->time;
  mj_resetDataKeyframe(mj_model_, mj_data_, keyframe_id);
  mj_data_->time = prev_sim_time;

  return true;
}

void MujocoSimulation::capture_initial_state()
{
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
  initial_qpos_.assign(mj_data_->qpos, mj_data_->qpos + mj_model_->nq);
  initial_qvel_.assign(mj_data_->qvel, mj_data_->qvel + mj_model_->nv);
  initial_ctrl_.assign(mj_data_->ctrl, mj_data_->ctrl + mj_model_->nu);
}

void MujocoSimulation::set_reset_callback(ResetCallback callback)
{
  reset_callback_ = std::move(callback);
}

void MujocoSimulation::start_physics_thread()
{
  // Disable the rangefinder flag at startup so that we don't get the yellow lines.
  // We can still turn this on manually if desired.
  sim_->opt.flags[mjVIS_RANGEFINDER] = false;
  // Turn off site rendering so that the visualization is more realistic.
  // These can still be turned on in the visualizer.
  for (int i = 0; i < mjNGROUP; i++)
  {
    sim_->opt.sitegroup[i] = 0;
  }

  // When the interface is activated, we start the physics engine.
  physics_thread_ = std::thread([this]() {
    // Load the simulation and do an initial forward pass
    RCLCPP_INFO(get_logger(), "Starting the MuJoCo physics thread...");
    if (this->headless_)
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
    physics_loop();
  });
}

void MujocoSimulation::shutdown()
{
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
}

void MujocoSimulation::reset_world_state(bool fill_initial_state)
{
  /// @note This method assumes sim_mutex_ is already held by the caller

  // Save the simulation time to preserve ROS clock continuity
  const mjtNum saved_time = mj_data_->time;

  if (fill_initial_state)
  {
    // Reset all positions, velocities and controls to initial state
    std::copy(initial_qpos_.begin(), initial_qpos_.end(), mj_data_->qpos);
    std::copy(initial_qvel_.begin(), initial_qvel_.end(), mj_data_->qvel);
    std::copy(initial_ctrl_.begin(), initial_ctrl_.end(), mj_data_->ctrl);
  }

  // Reset actuator activations (for muscles and similar)
  std::fill(mj_data_->act, mj_data_->act + mj_model_->na, 0.0);

  // Reset warmstart accelerations
  std::fill(mj_data_->qacc_warmstart, mj_data_->qacc_warmstart + mj_model_->nv, 0.0);

  // Reset sensor data
  std::fill(mj_data_->sensordata, mj_data_->sensordata + mj_model_->nsensordata, 0.0);

  // Reset actuator forces
  std::fill(mj_data_->actuator_force, mj_data_->actuator_force + mj_model_->nu, 0.0);

  // Reset applied forces
  std::fill(mj_data_->qfrc_applied, mj_data_->qfrc_applied + mj_model_->nv, 0.0);
  std::fill(mj_data_->xfrc_applied, mj_data_->xfrc_applied + 6 * mj_model_->nbody, 0.0);

  // Restore simulation time to preserve ROS clock continuity
  mj_data_->time = saved_time;

  // Run forward dynamics to update derived quantities
  mj_forward(mj_model_, mj_data_);

  // Copy to control data for reads - this ensures the physics loop uses the reset state
  mj_copyData(mj_data_control_, mj_model_, mj_data_);

  // Delegate HW-side bookkeeping (PID resets, command/state interface sync, etc.)
  if (reset_callback_)
  {
    reset_callback_(fill_initial_state);
  }
}

void MujocoSimulation::reset_world_callback(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetWorld::Request> request,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::ResetWorld::Response> response)
{
  RCLCPP_INFO_EXPRESSION(get_logger(), !request->keyframe.empty(), "Reset world service called with keyframe: '%s'",
                         request->keyframe.c_str());
  RCLCPP_INFO_EXPRESSION(get_logger(), request->keyframe.empty(),
                         "Reset world service called. Resetting to initial keyframe...");
  const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);

  bool fill_initial_state = request->keyframe.empty();
  if (!fill_initial_state)
  {
    if (!apply_keyframe(request->keyframe))
    {
      response->message = "Failed to apply keyframe: '" + request->keyframe + "'. Not resetting world.";
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      response->success = false;
      return;
    }
  }

  reset_world_state(fill_initial_state);
  response->success = true;
  const std::string keyframe_str = fill_initial_state ? "initial" : ("'" + request->keyframe + "'");
  response->message = "Successfully reset the MuJoCo world to the " + keyframe_str + " state.";

  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MujocoSimulation::set_pause_callback(const std::shared_ptr<mujoco_ros2_control_msgs::srv::SetPause::Request> request,
                                          std::shared_ptr<mujoco_ros2_control_msgs::srv::SetPause::Response> response)
{
  const bool currently_paused = !sim_->run;
  if (currently_paused == request->paused)
  {
    response->success = true;
    response->message = std::string("Simulation is already ") + (request->paused ? "paused." : "running.");
    RCLCPP_DEBUG(get_logger(), "%s", response->message.c_str());
    return;
  }

  sim_->run = !request->paused;

  if (!request->paused)
  {
    // Force timing re-sync so the physics loop doesn't try to catch up on
    // accumulated wall-clock time that elapsed while paused.
    sim_->speed_changed = true;

    // If step_simulation is currently blocking with pending steps, abort those steps
    // immediately rather than waiting for the physics loop to detect the transition.
    // This ensures step_simulation_callback unblocks and frees its executor thread
    // without any additional latency.
    const uint32_t pending = pending_steps_.load();
    if (pending > 0)
    {
      RCLCPP_WARN(get_logger(), "Resuming simulation while %u step(s) were pending; aborting.", pending);
      pending_steps_.store(0);
      steps_interrupted_.store(true);
      steps_cv_.notify_all();
    }
  }

  response->success = true;
  response->message = std::string("Simulation ") + (request->paused ? "paused." : "resumed.");
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MujocoSimulation::step_simulation_callback(
    const std::shared_ptr<mujoco_ros2_control_msgs::srv::StepSimulation::Request> request,
    std::shared_ptr<mujoco_ros2_control_msgs::srv::StepSimulation::Response> response)
{
  if (sim_->run)
  {
    response->success = false;
    response->message = "Cannot step simulation: simulation is not paused.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (request->steps == 0)
  {
    response->success = false;
    response->message = "Number of steps must be positive, got: 0";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  // Reset the divergence/interrupt flags and queue steps.
  step_diverged_.store(false);
  steps_interrupted_.store(false);
  pending_steps_.fetch_add(request->steps);

  // Block until all steps are executed or the timeout expires.
  // Timeout: at least 30 s, or 10 ms per step (whichever is larger).
  const auto timeout =
      std::chrono::milliseconds(std::max(static_cast<uint64_t>(30000), static_cast<uint64_t>(request->steps) * 10));

  std::unique_lock<std::mutex> lock(steps_cv_mutex_);
  const bool completed = steps_cv_.wait_for(lock, timeout, [this] { return pending_steps_.load() == 0; });

  if (!completed)
  {
    response->success = false;
    response->message = "Timeout waiting for " + std::to_string(request->steps) + " simulation step(s).";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }
  else if (steps_interrupted_.load())
  {
    response->success = false;
    response->message = "Steps aborted: simulation was resumed while steps were pending.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }
  else if (step_diverged_.load())
  {
    response->success = false;
    response->message = "Steps aborted: simulation diverged.";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }
  else
  {
    response->success = true;
    response->message = "Completed " + std::to_string(request->steps) + " simulation step(s).";
    RCLCPP_DEBUG(get_logger(), "%s", response->message.c_str());
  }
}

// simulate in background thread (while rendering in main thread)
void MujocoSimulation::physics_loop()
{
  // cpu-sim synchronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // Track previous simulation time to detect UI-triggered resets
  mjtNum prevSimTime = 0;

  // Initialize containers for data sharing
  // TODO: Update after breaking apart mujoco data.
  xfrc_viewer_capture_.assign(6 * mj_model_->nbody, 0.0);
  xfrc_plugin_desired_.assign(6 * mj_model_->nbody, 0.0);
  xfrc_last_restore_.assign(6 * mj_model_->nbody, 0.0);

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

      // Detect if a reset occurred via the UI (Sync() processed pending_.reset)
      // This is detected by simulation time jumping backwards to near-zero
      // The render loop's Sync() calls mj_resetData() which resets time to 0
      if (mj_model_ && mj_data_ && prevSimTime > 0.1 && mj_data_->time < 0.01)
      {
        RCLCPP_DEBUG(get_logger(), "UI reset detected (time jumped from %.3f to %.3f), applying initial state...",
                     prevSimTime, mj_data_->time);

        // Restore simulation time before reset_world_state saves it
        mj_data_->time = prevSimTime;

        // Apply initial state using common method
        reset_world_state(true);

        // Force speed_changed to re-sync timing
        sim_->speed_changed = true;

        RCLCPP_INFO(get_logger(), "Successfully applied initial state after UI reset.");
      }

      // run only if model is present
      if (mj_model_)
      {
        // Determine the viewer (drag) forces for this outer iteration.
        //
        // mjv_updateScene in simulate.cc reads mj_data_->xfrc_applied BEFORE zeroing it, so
        // plugin forces written here are visible as arrows in the native viewer.  To avoid
        // accumulation across outer iterations we must extract only the viewer-drag portion.
        //
        // After each outer iteration we restore mj_data_->xfrc_applied = viewer + plugin and
        // record it in xfrc_last_restore_.  If mj_data_->xfrc_applied still equals that value
        // the render thread has not run since our last step, so the viewer forces are unchanged
        // and xfrc_viewer_capture_ remains valid.  If it differs, the render thread ran: it
        // zeroed xfrc_applied and called mjv_applyPerturbForce, so mj_data_->xfrc_applied now
        // holds only drag forces — use that as the new viewer capture.
        const int nbody6 = 6 * mj_model_->nbody;
        if (std::memcmp(mj_data_->xfrc_applied, xfrc_last_restore_.data(), nbody6 * sizeof(mjtNum)) != 0)
        {
          // Render thread ran: xfrc_applied was zeroed then drag was applied.
          mju_copy(xfrc_viewer_capture_.data(), mj_data_->xfrc_applied, nbody6);
        }
        // else: render thread did not run; keep the existing xfrc_viewer_capture_.

        // running
        if (sim_->run)
        {
          // If the sim was unpaused while a StepSimulation call was in progress,
          // abort the remaining pending steps so the service call unblocks cleanly.
          if (pending_steps_.load() > 0)
          {
            RCLCPP_WARN(get_logger(), "Simulation resumed while %u step(s) were still pending; aborting.",
                        pending_steps_.load());
            pending_steps_.store(0);
            steps_interrupted_.store(true);
            steps_cv_.notify_all();
          }

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
            mju_copy(mj_data_->ctrl, mj_data_control_->ctrl, static_cast<int>(mj_model_->nu));
            mju_copy(mj_data_->qfrc_applied, mj_data_control_->qfrc_applied, static_cast<int>(mj_model_->nu));

            // Restore viewer forces, then add plugin contribution on top.
            // TODO: Cleanup when mujoco data is split
            mju_copy(mj_data_->xfrc_applied, xfrc_viewer_capture_.data(), nbody6);
            mju_addTo(mj_data_->xfrc_applied, xfrc_plugin_desired_.data(), nbody6);

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
              step_count_.fetch_add(1);
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
              mju_copy(mj_data_->ctrl, mj_data_control_->ctrl, static_cast<int>(mj_model_->nu));
              mju_copy(mj_data_->qfrc_applied, mj_data_control_->qfrc_applied, static_cast<int>(mj_model_->nu));

              // Restore viewer forces, then add plugin contribution (xfrc_plugin_desired_ is only
              // written by the control thread between outer loop iterations — constant here).
              // TODO: Cleanup when mujoco data is split
              mju_copy(mj_data_->xfrc_applied, xfrc_viewer_capture_.data(), nbody6);
              mju_addTo(mj_data_->xfrc_applied, xfrc_plugin_desired_.data(), nbody6);

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
                step_count_.fetch_add(1);
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

            // Restore viewer + plugin into mj_data_->xfrc_applied so mjv_updateScene (which
            // reads it before zeroing) shows plugin force arrows in the native viewer.
            // Record the value in xfrc_last_restore_ so the next outer iteration can detect
            // whether the render thread ran and updated xfrc_applied in the interim.
            // TODO: Cleanup when mujoco data is split
            mju_copy(mj_data_->xfrc_applied, xfrc_viewer_capture_.data(), nbody6);
            mju_addTo(mj_data_->xfrc_applied, xfrc_plugin_desired_.data(), nbody6);
            mju_copy(xfrc_last_restore_.data(), mj_data_->xfrc_applied, nbody6);

            sim_->AddToHistory();
            update_sim_display();
          }
        }

        // paused
        else
        {
          // Translate keyboard 'S' presses into single pending steps.
          if (keyboard_step_requested_.exchange(false))
          {
            step_diverged_.store(false);
            pending_steps_.fetch_add(1);
          }

          // Execute one pending step per physics loop iteration so the clock publisher
          // (try_publish) has time to flush between steps, matching play mode behavior.
          if (pending_steps_.load() > 0)
          {
            mju_copy(mj_data_->ctrl, mj_data_control_->ctrl, static_cast<int>(mj_model_->nu));
            mju_copy(mj_data_->qfrc_applied, mj_data_control_->qfrc_applied, static_cast<int>(mj_model_->nu));

            // TODO: Cleanup when mujoco data is split
            mju_copy(mj_data_->xfrc_applied, xfrc_viewer_capture_.data(), nbody6);
            mju_addTo(mj_data_->xfrc_applied, xfrc_plugin_desired_.data(), nbody6);

            mj_step(mj_model_, mj_data_);
            publish_clock();

            const char* message = Diverged(mj_model_->opt.disableflags, mj_data_);
            if (message)
            {
              pending_steps_.store(0);
              step_diverged_.store(true);
              mju::strcpy_arr(sim_->load_error, message);
              steps_cv_.notify_all();
            }
            else
            {
              mj_copyData(mj_data_control_, mj_model_, mj_data_);

              // TODO: Cleanup when mujoco data is split
              mju_copy(mj_data_->xfrc_applied, xfrc_viewer_capture_.data(), nbody6);
              mju_addTo(mj_data_->xfrc_applied, xfrc_plugin_desired_.data(), nbody6);
              mju_copy(xfrc_last_restore_.data(), mj_data_->xfrc_applied, nbody6);

              sim_->AddToHistory();
              pending_steps_.fetch_sub(1);
              step_count_.fetch_add(1);
              steps_cv_.notify_all();
              update_sim_display();
            }

            // Force timing re-sync when/if simulation is resumed
            if (pending_steps_.load() == 0)
            {
              sim_->speed_changed = true;
            }
          }
          else
          {
            mj_copyData(mj_data_control_, mj_model_, mj_data_);

            // TODO: Cleanup when mujoco data is split
            mju_copy(mj_data_->xfrc_applied, xfrc_viewer_capture_.data(), nbody6);
            mju_addTo(mj_data_->xfrc_applied, xfrc_plugin_desired_.data(), nbody6);
            mju_copy(xfrc_last_restore_.data(), mj_data_->xfrc_applied, nbody6);

            // run mj_forward, to update rendering and joint sliders
            mj_forward(mj_model_, mj_data_);
            sim_->speed_changed = true;
            update_sim_display();
          }
        }

        // Update previous simulation time for next iteration
        if (mj_data_)
        {
          prevSimTime = mj_data_->time;
        }
      }
    }  // release std::lock_guard<std::mutex>
  }
}

void MujocoSimulation::publish_clock()
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

void MujocoSimulation::update_sim_display()
{
  if (headless_)
  {
    return;
  }

  // Only write user_texts_new_ when the render thread has consumed the previous
  // update (newtextrequest == 0). Use compare_exchange to atomically claim the
  // slot: if it fails, the render thread hasn't swapped yet, so skip this
  // update — the display will be refreshed on the next physics step instead.
  // This avoids a data race: the render thread swaps user_texts_new_ (without
  // holding any mutex) while we clear/populate it.
  int expected = 0;
  if (!sim_->newtextrequest.compare_exchange_strong(expected, 1))
  {
    return;  // render thread hasn't consumed the last update yet, skip
  }

  const std::string status = sim_->run ? "Running" : "Paused";
  sim_->user_texts_new_.clear();
  sim_->user_texts_new_.emplace_back(mjFONT_NORMAL, mjGRID_TOPRIGHT, "Status\nSteps",
                                     status + "\n" + std::to_string(step_count_.load()));
}

}  // namespace mujoco_ros2_control
