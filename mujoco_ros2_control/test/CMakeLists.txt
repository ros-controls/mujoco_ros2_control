find_package(ament_cmake_gtest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(control_toolbox REQUIRED)

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
  Eigen3::Eigen
  control_toolbox::control_toolbox
)
