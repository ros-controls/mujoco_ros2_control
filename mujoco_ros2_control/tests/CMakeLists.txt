find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(test_plugin test_plugin.cpp)
target_link_libraries(test_plugin
  mujoco_ros2_control
  rclcpp::rclcpp
)

ament_add_gtest(test_headless_init test_headless_init.cpp)
target_link_libraries(test_headless_init
  mujoco_ros2_control
  rclcpp::rclcpp
  hardware_interface::hardware_interface
)

ament_add_pytest_test(test_urdf_to_mujoco_utils test_urdf_to_mujoco_utils.py)
