cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control)

find_package(ros2_control_cmake REQUIRED)

# find dependencies
# ROS2 package dependencies (matching package.xml)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  backward_ros
  control_toolbox
  controller_manager
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  mujoco_ros2_control_msgs
  transmission_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Non-ROS2 dependencies
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Attempt to find an install of mujoco, either the library itself, from the environment, or
# as a last resort download the required tar file manually and unzip it into the workspace.
# Note: When MuJoCo is fetched via FetchContent, MUJOCO_SIMULATE_DIR (build interface)
# and MUJOCO_INSTALL_SIMULATE_DIR (install interface) differ - build uses source tree,
# install uses install prefix. For system-installed MuJoCo, these paths are the same.
find_package(mujoco QUIET)
if(mujoco_FOUND)
  message(STATUS "Mujoco build from source has been found")
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
  set(MUJOCO_SIMULATE_DIR ${MUJOCO_INCLUDE_DIR}/../simulate)
  # For installed mujoco, use the provided paths
  # Needed for generator expressions in exported targets ($<INSTALL_INTERFACE:...>)
  set(MUJOCO_INSTALL_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
  set(MUJOCO_INSTALL_SIMULATE_DIR "${MUJOCO_SIMULATE_DIR}")
elseif(DEFINED ENV{MUJOCO_INSTALL_DIR})
  message(STATUS "Mujoco build from source has not been found. Attempting to find the binary in $ENV{MUJOCO_INSTALL_DIR} instead.")
  set(MUJOCO_DIR $ENV{MUJOCO_INSTALL_DIR})
  find_library(MUJOCO_LIB mujoco HINTS $ENV{MUJOCO_DIR}/lib)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find binary in $ENV{MUJOCO_DIR}")
  endif()
  set(MUJOCO_INCLUDE_DIR $ENV{MUJOCO_DIR}/include)
  set(MUJOCO_SIMULATE_DIR $ENV{MUJOCO_DIR}/simulate)
  # For installed mujoco, use the provided paths
  # Needed for generator expressions in exported targets ($<INSTALL_INTERFACE:...>)
  set(MUJOCO_INSTALL_INCLUDE_DIR "${MUJOCO_INCLUDE_DIR}")
  set(MUJOCO_INSTALL_SIMULATE_DIR "${MUJOCO_SIMULATE_DIR}")
else()
  include(FetchContent)

  # Detect architecture
  set(MUJOCO_VERSION "3.4.0")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
    set(CPU_ARCH "x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(CPU_ARCH "aarch64")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  message(STATUS "No MuJoCo installation found. Downloading MuJoCo ${MUJOCO_VERSION} for ${CPU_ARCH}.")

  # Download the archive and put it next to the source directory
  set(MUJOCO_DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mujoco)
  set(MUJOCO_EXTRACT_DIR ${MUJOCO_DOWNLOAD_DIR}/mujoco-${MUJOCO_VERSION})
  set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
  FetchContent_Declare(
    mujoco_download
    SOURCE_DIR ${MUJOCO_EXTRACT_DIR}
    DOWNLOAD_EXTRACT_TIMESTAMP True
    URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${CPU_ARCH}.tar.gz
  )
  FetchContent_MakeAvailable(mujoco_download)

  set(MUJOCO_DIR ${MUJOCO_EXTRACT_DIR})
  message(STATUS "MuJoCo downloaded to: ${MUJOCO_DIR}")

  # Find the library in the downloaded location
  find_library(MUJOCO_LIB mujoco HINTS ${MUJOCO_DIR}/lib NO_DEFAULT_PATH)
  if(NOT MUJOCO_LIB)
    message(FATAL_ERROR "Failed to find MuJoCo library in ${MUJOCO_DIR}/lib")
  endif()

  set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
  set(MUJOCO_SIMULATE_DIR ${MUJOCO_DIR}/simulate)

  message(STATUS "Installing ${MUJOCO_LIB} to the install directory...")
  install(
    DIRECTORY ${MUJOCO_DIR}/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "libmujoco.so*"
  )
  # Install mujoco headers so they're available when exporting targets
  install(
    DIRECTORY ${MUJOCO_INCLUDE_DIR}/
    DESTINATION include/mujoco
  )
  install(
    DIRECTORY ${MUJOCO_SIMULATE_DIR}/
    DESTINATION include/mujoco_simulate
  )
  # Set install paths for use in generator expressions ($<INSTALL_INTERFACE:...>)
  # Enables exported targets to work when package is installed and consumed by others
  set(MUJOCO_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/mujoco")
  set(MUJOCO_INSTALL_SIMULATE_DIR "${CMAKE_INSTALL_PREFIX}/include/mujoco_simulate")
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
    set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC
      $<BUILD_INTERFACE:${lodepng_SOURCE_DIR}>
      $<INSTALL_INTERFACE:include>
    )
    # Install lodepng header
    install(FILES ${lodepng_SOURCE_DIR}/lodepng.h
      DESTINATION include
    )
  endif()
endif()

# We don't care about warnings from the external sources
set(MUJOCO_SILENCE_COMPILER_WARNINGS
  -Wno-missing-field-initializers
  -Wno-unused-parameter
  -Wno-sign-compare
  -Wno-psabi
)

# Build Mujoco's simulate application and make it available for linking
add_library(platform_ui_adapter OBJECT)
set_target_properties(platform_ui_adapter PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_sources(
  platform_ui_adapter
  PRIVATE ${MUJOCO_SIMULATE_DIR}/glfw_adapter.h ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.h ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.h
  PRIVATE ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
)
target_include_directories(
  platform_ui_adapter PUBLIC
    $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
    $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${MUJOCO_INSTALL_SIMULATE_DIR}>
    $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
)
target_link_libraries(platform_ui_adapter PUBLIC ${MUJOCO_LIB} glfw)
target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
add_library(mujoco::libsimulate ALIAS libsimulate)
set_target_properties(libsimulate PROPERTIES
  OUTPUT_NAME simulate
  POSITION_INDEPENDENT_CODE ON
)
target_sources(
  libsimulate
  PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.h ${MUJOCO_SIMULATE_DIR}/simulate.cc ${MUJOCO_SIMULATE_DIR}/array_safety.h
)
target_include_directories(libsimulate PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_SIMULATE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
)
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter ${MUJOCO_LIB})
target_compile_options(libsimulate PRIVATE ${MUJOCO_SILENCE_COMPILER_WARNINGS})

# We set these after creating the targets to avoid propagating them to other third-party targets
set_compiler_options()
export_windows_symbols()

# Mujoco ROS 2 control system interface (this package)
add_library(mujoco_ros2_control SHARED
    src/mujoco_system_interface.cpp
    src/mujoco_cameras.cpp
    src/mujoco_lidar.cpp
)
# Ensure MuJoCo library can be found at runtime regardless of how it was installed above
set_target_properties(mujoco_ros2_control PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib;${CMAKE_INSTALL_PREFIX}/lib"
)
target_link_libraries(mujoco_ros2_control
  ${MUJOCO_LIB}
  ${sensor_msgs_TARGETS}
  mujoco::libsimulate
  controller_manager::controller_manager
  hardware_interface::hardware_interface
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  control_toolbox::control_toolbox
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mujoco_ros2_control_msgs_TARGETS}
  transmission_interface::transmission_interface
  Eigen3::Eigen
  Threads::Threads
  lodepng
  glfw
)
target_include_directories(mujoco_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# Mark these as system packages to prevent compiler warnings
target_include_directories(mujoco_ros2_control SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_SIMULATE_DIR}>
)
install(TARGETS mujoco_ros2_control libsimulate lodepng platform_ui_adapter
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

# Install the ROS 2 control node
add_executable(ros2_control_node src/mujoco_ros2_control_node.cpp)
target_link_libraries(ros2_control_node PUBLIC
  controller_manager::controller_manager
)
install(
  TARGETS ros2_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install python tools
install(PROGRAMS
  scripts/find_missing_inertias.py
  scripts/make_mjcf_from_robot_description.py
  scripts/robot_description_to_mjcf.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY resources scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_interface_plugin.xml)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

## EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
