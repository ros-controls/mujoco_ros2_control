find_package(ament_cmake_gtest REQUIRED)

install(
  DIRECTORY launch test_resources config
  DESTINATION share/${PROJECT_NAME}
)

ament_add_gtest(test_plugin test_plugin.cpp)
target_link_libraries(test_plugin
  mujoco_ros2_control
  rclcpp::rclcpp
)

find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(src/robot_launch_test.py
  TIMEOUT 50
)

add_launch_test(src/robot_launch_pid_test.py
  TIMEOUT 50
)

add_launch_test(src/robot_launch_mjcf_generation_test.py
  TIMEOUT 50
)

add_launch_test(src/robot_launch_mjcf_generation_from_robot_description_test.py
  TIMEOUT 50
)

add_launch_test(src/robot_launch_transmission_test.py
  TIMEOUT 50
)

add_launch_test(src/robot_launch_transmission_pid_test.py
  TIMEOUT 50
)
