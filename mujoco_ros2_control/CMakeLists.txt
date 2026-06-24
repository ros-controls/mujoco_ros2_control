cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control)

find_package(ros2_control_cmake REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL)

find_package(control_toolbox REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(Threads REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(backward_ros REQUIRED)
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 REQUIRED)

# Link MuJoCo via the vendor package
find_package(mujoco_vendor REQUIRED)

# MuJoCo ROS 2 control system interface (this package)
add_library(mujoco_ros2_control SHARED
    src/mujoco_simulation.cpp
    src/mujoco_system_interface.cpp
)
set_compiler_options()
export_windows_symbols()
# Ensure MuJoCo library can be found at runtime regardless of how it was installed above.
# mujoco_vendor installs libmujoco.so under opt/mujoco_vendor/lib/ relative to the
# install prefix, so that directory must be in the RPATH alongside the standard lib/.
set_target_properties(mujoco_ros2_control PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../opt/mujoco_vendor/lib;${CMAKE_INSTALL_PREFIX}/lib"
)
# MuJoCo has TinyXML2 statically linked, which conflicts with the system TinyXML2
# used by FastDDS. We need to ensure the system TinyXML2 is loaded first.
# Force tinyxml2 to appear first in the DT_NEEDED entries by using LINK_OPTIONS.
# TODO: Remove this after the following is resolved and the next ROS sync
# https://github.com/pal-robotics/mujoco_vendor/issues/2
target_link_options(mujoco_ros2_control PUBLIC
  "LINKER:--push-state,--no-as-needed"
  "$<TARGET_LINKER_FILE:tinyxml2::tinyxml2>"
  "LINKER:--pop-state"
)
target_link_libraries(mujoco_ros2_control
PUBLIC
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${mujoco_ros2_control_msgs_TARGETS}
  pluginlib::pluginlib
  fmt::fmt
  mujoco::mujoco
  control_toolbox::control_toolbox
  Eigen3::Eigen
PRIVATE
  mujoco::simulate
  Threads::Threads
  lodepng::lodepng
  glfw
  OpenGL::EGL
  OpenGL::OpenGL
  ${CMAKE_DL_LIBS}
)

# Note: the transmissions_interface must NOT be statically linked against the library, only headers
# should be used. Otherwise it will be loaded by the linker at process startup and its
# PLUGINLIB_EXPORT_CLASS static constructors will have run before the ClassLoaders exist. This
# can cause problems when pluginlib calls dlopen, as a previously-loaded handle will be
# returned that will prevent registration of a factory for the transmission type.
target_include_directories(mujoco_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_SIMULATE_DIR}>
  $<TARGET_PROPERTY:transmission_interface::transmission_interface,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:mujoco_ros2_control_plugins::mujoco_ros2_control_plugins,INTERFACE_INCLUDE_DIRECTORIES>
)
# Mark MuJoCo source as system to avoid compiler warnings
target_include_directories(mujoco_ros2_control SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
)

# Note: mujoco::mujoco is an IMPORTED target owned by mujoco_vendor and cannot be
# co-installed here. Downstream packages must find_package(mujoco_vendor) directly,
# which is ensured via ament_export_dependencies below.
install(TARGETS mujoco_ros2_control
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

# Install the ROS 2 control node
add_executable(ros2_control_node src/mujoco_ros2_control_node.cpp)
target_link_libraries(ros2_control_node PUBLIC
  controller_manager::controller_manager
)
install(
  TARGETS ros2_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install python tools
install(PROGRAMS
  scripts/find_missing_inertias.py
  scripts/make_mjcf_from_robot_description.py
  scripts/robot_description_to_mjcf.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY resources scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  FILES scripts/requirements.txt
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_interface_plugin.xml)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_python_install_package(mujoco_ros2_control
  SCRIPTS_DESTINATION lib/mujoco_ros2_control
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  transmission_interface
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  mujoco_ros2_control_msgs
  mujoco_ros2_control_plugins
  pluginlib
  control_toolbox
  Eigen3
  mujoco_vendor
)
ament_package()
