cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control_demos)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config demo_resources
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
