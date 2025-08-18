find_package(ament_cmake_gtest REQUIRED)

install(
  DIRECTORY launch test_resources config
  DESTINATION share/${PROJECT_NAME}
)

ament_add_gtest(test_plugin test_plugin.cpp)
target_link_libraries(test_plugin mujoco_ros2_simulation)
ament_target_dependencies(test_plugin
  rclcpp
  ament_cmake_gtest
)
