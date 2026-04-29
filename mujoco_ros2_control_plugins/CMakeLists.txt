cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control_plugins)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)

# Attempt to link MuJoCo via the vendor package, if available
find_package(mujoco_vendor QUIET)
if(mujoco_vendor_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${mujoco_vendor_INCLUDE_DIRS})
else()
  # Fallback to system MuJoCo or other methods if needed
  message(FATAL_ERROR "mujoco_vendor not found!")
endif()


# Build the plugin library (all plugins compiled into one shared library)
add_library(mujoco_ros2_control_plugins SHARED
  src/heartbeat_publisher_plugin.cpp
  src/external_wrench_plugin.cpp
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
  realtime_tools::realtime_tools
  pluginlib::pluginlib
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${mujoco_ros2_control_msgs_TARGETS}
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

  ament_add_gtest(test_external_wrench_plugin
    test/test_external_wrench_plugin.cpp
  )
  target_link_libraries(test_external_wrench_plugin
    mujoco_ros2_control_plugins
    ${mujoco_ros2_control_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  )
  # MuJoCo has TinyXML2 statically linked, which conflicts with the system TinyXML2
  # used by FastDDS. We need to ensure the system TinyXML2 is loaded first.
  # Force tinyxml2 to appear first in the DT_NEEDED entries by using LINK_OPTIONS.
  # TODO: Remove this after the following is resolved and the next ROS sync
  # https://github.com/pal-robotics/mujoco_vendor/issues/2
  target_link_options(test_external_wrench_plugin PUBLIC
    "LINKER:--push-state,--no-as-needed"
    "$<TARGET_LINKER_FILE:tinyxml2::tinyxml2>"
    "LINKER:--pop-state"
  )

endif()

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Export plugin description
pluginlib_export_plugin_description_file(mujoco_ros2_control_plugins mujoco_ros2_control_plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(mujoco_vendor rclcpp realtime_tools pluginlib std_msgs visualization_msgs mujoco_ros2_control_msgs ros2_control_cmake)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
