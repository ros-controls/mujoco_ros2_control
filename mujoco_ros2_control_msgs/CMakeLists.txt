cmake_minimum_required(VERSION 3.10)

project(mujoco_ros2_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  msg/ExternalWrench.msg
  msg/ExternalWrenchArray.msg
)

set(srv_files
  srv/ResetWorld.srv
  srv/SetPause.srv
  srv/StepSimulation.srv
  srv/ApplyExternalWrench.srv
)


rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
