cmake_minimum_required(VERSION 3.8)

project(mujoco_ros2_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(glfw3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
elseif(DEFINED ENV{MUJOCO_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_DIR} instead.")
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
else()
  message(FATAL_ERROR "Failed to find mujoco with find_package.
  Either build and install mujoco from source or set the MUJOCO_DIR environment variable to tell CMake where to find the binary install. ")
endif()

# Fetch lodepng dependency.
if(NOT TARGET lodepng)
  include(FetchContent)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

# add simulate things
set(SIMULATE_DIR $ENV{MUJOCO_DIR}/simulate)

# Simulate library
add_library(platform_ui_adapter OBJECT)
set_target_properties(platform_ui_adapter PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_sources(
  platform_ui_adapter
  PUBLIC ${SIMULATE_DIR}/glfw_adapter.h ${SIMULATE_DIR}/glfw_dispatch.h ${SIMULATE_DIR}/platform_ui_adapter.h
  PRIVATE ${SIMULATE_DIR}/glfw_adapter.cc ${SIMULATE_DIR}/glfw_dispatch.cc ${SIMULATE_DIR}/platform_ui_adapter.cc
)
target_include_directories(
  platform_ui_adapter PUBLIC ${SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR}
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC ${MUJOCO_LIB})
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
add_library(mujoco::libsimulate ALIAS libsimulate)
set_target_properties(libsimulate PROPERTIES
  OUTPUT_NAME simulate
  POSITION_INDEPENDENT_CODE ON
)
target_sources(
  libsimulate
  PUBLIC ${SIMULATE_DIR}/simulate.h
  PRIVATE ${SIMULATE_DIR}/simulate.cc ${SIMULATE_DIR}/array_safety.h
)
target_include_directories(libsimulate PUBLIC ${SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter ${MUJOCO_LIB})
target_compile_options(libsimulate PRIVATE -Wno-missing-field-initializers)

# Mujoco ROS 2 control system interface (this package)
add_library(mujoco_ros2_simulation SHARED
    src/mujoco_system_interface.cpp
    src/mujoco_simulate.cpp
)
ament_target_dependencies(mujoco_ros2_simulation
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
target_link_libraries(mujoco_ros2_simulation
  ${MUJOCO_LIB}
  mujoco::libsimulate
  Threads::Threads
  lodepng
  glfw
)
target_include_directories(mujoco_ros2_simulation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)
install(TARGETS mujoco_ros2_simulation
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_interface_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()