cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(mujoco_ros2_control REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  install(
    DIRECTORY launch config test_resources
    DESTINATION share/${PROJECT_NAME}
  )

  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/robot_launch_test.py
    TIMEOUT 50
  )

  add_launch_test(test/robot_launch_pid_test.py
    TIMEOUT 50
  )

  add_launch_test(test/robot_launch_mjcf_generation_test.py
    TIMEOUT 50
  )

  add_launch_test(test/robot_launch_mjcf_generation_from_robot_description_test.py
    TIMEOUT 50
  )

  add_launch_test(test/robot_launch_transmission_test.py
    TIMEOUT 50
  )

  add_launch_test(test/robot_launch_transmission_pid_test.py
    TIMEOUT 50
  )
endif()

ament_package()
