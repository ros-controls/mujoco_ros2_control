#include "mujoco_ros2_simulation/mujoco_system_interface.hpp"
#include "mujoco_ros2_simulation/mujoco_simulate.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace mujoco_ros2_simulation
{

MujocoSystemInterface::MujocoSystemInterface() = default;

MujocoSystemInterface::~MujocoSystemInterface()
{
  // If sim_ is created and running, clean shut it down
  if (sim_) {
    sim_->exitrequest.store(true);

    if (physics_thread_.joinable()) {
      physics_thread_.join();
    }
    if (ui_thread_.joinable()) {
      ui_thread_.join();
    }
  }

  // Cleanup data and the model, if they haven't been
  if (data_) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  system_info_ = info;

  // Load the model path from hardware parameters
  if (info.hardware_parameters.count("mujoco_model") == 0) {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Missing 'mujoco_model' in <hardware_parameters>.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  model_path_ = info.hardware_parameters.at("mujoco_model");

  RCLCPP_INFO_STREAM(rclcpp::get_logger("MujocoSystemInterface"), "Loading 'mujoco_model' from: " << model_path_);

  // We essentially reconstruct the 'simulate.cc::main()' function here, and launch a Simulate
  // object with all necessary rendering process/options attached.

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  sim_ = std::make_unique<mujoco::Simulate>(
    std::make_unique<mujoco::GlfwAdapter>(),
    &cam, &opt, &pert, /* is_passive = */ false
  );

  // We maintain a pointer to the mutex so that we can lock from here, too.
  // Is this a terrible idea? Maybe, but it lets us use their libraries as is...
  sim_mutex_ = &sim_->mtx;

  // From here, we wrap up the PhysicsThread's starting function before beginning the loop in activate
  sim_->LoadMessage(model_path_.c_str());
  model_ = LoadModel(model_path_.c_str(), *sim_);
  if (!model_) {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Mujoco failed to load '%s'", model_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  {
    std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
    data_ = mj_makeData(model_);
  }
  if (!data_) {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoSystemInterface"), "Could not allocate mjData for '%s'", model_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get joint information and map it to our pointers
  n_joints_ = system_info_.joints.size();
  joint_names_.resize(n_joints_);
  muj_joint_id_.resize(n_joints_);
  muj_actuator_id_.resize(n_joints_);
  hw_positions_.assign(n_joints_, 0.0);
  hw_velocities_.assign(n_joints_, 0.0);
  hw_efforts_.assign(n_joints_, 0.0);
  hw_commands_.assign(n_joints_, 0.0);

  for (size_t i = 0; i < n_joints_; ++i) {
    joint_names_[i] = system_info_.joints[i].name;

    // Precompute joint_id and actuator_id for faster read/write:
    muj_joint_id_[i] = mj_name2id(model_, mjOBJ_JOINT, joint_names_[i].c_str());
    if (muj_joint_id_[i] < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MujocoSystemInterface"),
        "Joint '%s' not found in MuJoCo model", joint_names_[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    muj_actuator_id_[i] = mj_name2id(model_, mjOBJ_ACTUATOR, joint_names_[i].c_str());
    if (muj_actuator_id_[i] < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MujocoSystemInterface"),
        "Actuator '%s' not found in MuJoCo model", joint_names_[i].c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MujocoSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(n_joints_ * 3);

  for (size_t i = 0; i < n_joints_; ++i)
  {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MujocoSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(n_joints_);

  // TODO: Other command types?
  for (size_t i = 0; i < n_joints_; ++i)
  {
    command_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
              "Activating MuJoCo hardware interface and starting Simulate threads...");

  // Start the physics thread in the background, which will load the simulation and wait for rendering to begin
  physics_thread_ = std::thread([this]() {
    // Load the simulation and do an initial forward pass
    RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Starting the mujoco physics thread...");
    sim_->Load(model_, data_, model_path_.c_str());
    // lock the sim mutex
    {
      const std::unique_lock<std::recursive_mutex> lock(*sim_mutex_);
      mj_forward(model_, data_);
    }
    // Blocks until terminated
    PhysicsLoop(*sim_);
  });

  // Launch the UI loop in the background
  ui_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"), "Starting the mujoco rendering thread...");
    // Blocks until terminated
    sim_->RenderLoop();
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystemInterface"),
              "Deactivating MuJoCo hardware interface and shutting down Simulate...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystemInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystemInterface::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

} // end namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_simulation::MujocoSystemInterface,
  hardware_interface::SystemInterface
);
