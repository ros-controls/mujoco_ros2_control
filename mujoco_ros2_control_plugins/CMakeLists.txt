cmake_minimum_required(VERSION 3.16)
project(mujoco_ros2_control_plugins)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(rclcpp REQUIRED)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # find_package(ament_lint_auto REQUIRED)
  # ament_lint_auto_find_test_dependencies()
endif()

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(mujoco_vendor rclcpp ros2_control_cmake)

ament_package()
