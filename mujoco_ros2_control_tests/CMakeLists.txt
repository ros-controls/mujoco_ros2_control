cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)

  install(
    DIRECTORY launch config test_resources
    DESTINATION share/${PROJECT_NAME}
  )

  find_package(launch_testing_ament_cmake REQUIRED)

  # It is unclear why, but ctest in github action runners doesn't seem to correctly inherit
  # the PYTHONPATH from `install/setup.bash`. So we provide the exact path to ensure these are
  # locatable in CI.
  set(append_env_args "")
  if($ENV{GITHUB_ACTIONS})
    set(test_pythonpath "$ENV{PYTHONPATH}")
    file(TO_CMAKE_PATH "${test_pythonpath}" test_pythonpath)
    list(APPEND append_env_args APPEND_ENV "PYTHONPATH=${test_pythonpath}")
  endif()

  add_launch_test(test/robot_launch_test.py
    TIMEOUT 50
    ${append_env_args}
  )

  add_launch_test(test/robot_launch_pid_test.py
    TIMEOUT 50
    ${append_env_args}
  )

  add_launch_test(test/robot_launch_mjcf_generation_test.py
    TIMEOUT 50
    ${append_env_args}
  )

  add_launch_test(test/robot_launch_mjcf_generation_from_robot_description_test.py
    TIMEOUT 50
    ${append_env_args}
  )

  add_launch_test(test/robot_launch_transmission_test.py
    TIMEOUT 50
    ${append_env_args}
  )

  add_launch_test(test/robot_launch_transmission_pid_test.py
    TIMEOUT 50
    ${append_env_args}
  )
endif()

ament_package()
