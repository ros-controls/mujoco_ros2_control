cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control_plugins)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# Attempt to link MuJoCo via the vendor package, if available
find_package(mujoco_vendor QUIET)
if(mujoco_vendor_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${mujoco_vendor_INCLUDE_DIRS})
else()
  # Fallback to system MuJoCo or other methods if needed
  message(FATAL_ERROR "mujoco_vendor not found!")
endif()


# Build the heartbeat publisher plugin
add_library(mujoco_ros2_control_plugins SHARED
  src/heartbeat_publisher_plugin.cpp
)
target_include_directories(mujoco_ros2_control_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Mark MuJoCo source as system to avoid compiler warnings
target_include_directories(mujoco_ros2_control_plugins SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
)
target_link_libraries(mujoco_ros2_control_plugins PUBLIC
  ${MUJOCO_LIB}
  rclcpp::rclcpp
  pluginlib::pluginlib
  ${std_msgs_TARGETS}
)

# Install the plugin library
install(TARGETS mujoco_ros2_control_plugins
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Export plugin description
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins mujoco_ros2_control_plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(mujoco_vendor rclcpp pluginlib std_msgs ros2_control_cmake)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
