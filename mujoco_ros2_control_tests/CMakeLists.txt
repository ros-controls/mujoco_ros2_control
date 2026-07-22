cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(mujoco_ros2_control_plugins REQUIRED)

  # Compile a test plugin alongside base plugins to make sure it works.
  add_library(test_plugin SHARED src/test_plugin.cpp)
  target_link_libraries(test_plugin
    ${mujoco_ros2_control_plugins_TARGETS}
    pluginlib::pluginlib
  )

  install(TARGETS test_plugin
    LIBRARY DESTINATION lib
  )

  pluginlib_export_plugin_description_file(
    mujoco_ros2_control_plugins src/test_plugin.xml
  )

  install(
    DIRECTORY launch config test_resources
    DESTINATION share/${PROJECT_NAME}
  )

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Run each launch test with a unique ROS_DOMAIN_ID so concurrent or
  # tearing-down test runs cannot cross-talk over DDS.
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()

  add_ros_isolated_launch_test(test/robot_launch_test.py
    TIMEOUT 75
  )

  add_ros_isolated_launch_test(test/robot_launch_pid_test.py
    TIMEOUT 75
  )

  add_ros_isolated_launch_test(test/robot_launch_mjcf_generation_test.py
    TIMEOUT 250
  )

  add_ros_isolated_launch_test(test/robot_launch_mjcf_generation_from_robot_description_test.py
    TIMEOUT 250
  )

  add_ros_isolated_launch_test(test/robot_launch_transmission_test.py
    TIMEOUT 75
  )

  add_ros_isolated_launch_test(test/robot_launch_transmission_pid_test.py
    TIMEOUT 75
  )

  add_ros_isolated_launch_test(test/site_velocity_launch_test.py
    TIMEOUT 75
  )
endif()

ament_package()
