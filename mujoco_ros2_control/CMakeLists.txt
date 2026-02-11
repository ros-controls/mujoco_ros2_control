cmake_minimum_required(VERSION 3.16)

project(mujoco_ros2_control)

find_package(ros2_control_cmake REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED COMPONENTS EGL)

# Find OSMesa for headless software rendering fallback
find_library(OSMESA_LIBRARY OSMesa)
find_path(OSMESA_INCLUDE_DIR GL/osmesa.h)
if(OSMESA_LIBRARY AND OSMESA_INCLUDE_DIR)
  message(STATUS "Found OSMesa: ${OSMESA_LIBRARY}")
  set(OSMESA_FOUND TRUE)
else()
  message(STATUS "OSMesa not found - headless software rendering fallback will be disabled")
  set(OSMESA_FOUND FALSE)
endif()
find_package(control_toolbox REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(backward_ros REQUIRED)
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 REQUIRED)

# Attempt to link MuJoCo via the vendor package, if available
find_package(mujoco_vendor QUIET)
if(mujoco_vendor_FOUND)
  set(MUJOCO_LIB mujoco::mujoco)
  set(MUJOCO_INCLUDE_DIR ${mujoco_vendor_INCLUDE_DIRS})
  set(MUJOCO_SIMULATE_DIR ${MUJOCO_INCLUDE_DIR}/../simulate)
  set(MUJOCO_INSTALL_INCLUDE_DIR ${MUJOCO_INCLUDE_DIR})
  set(MUJOCO_INSTALL_SIMULATE_DIR ${MUJOCO_SIMULATE_DIR})
  set(MUJOCO_EXPORT_TARGETS "")
else()
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  # Otherwise we fallback to downloading the required version on the fly, but this requires
  # an active internet connection! This should only be necessary for environments that do
  # not have the vendor package available (like pixi, currently).
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  include(DownloadMujoco)
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
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
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
target_include_directories(platform_ui_adapter PUBLIC ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
target_link_libraries(platform_ui_adapter PRIVATE ${MUJOCO_LIB} glfw)
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
  PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.h
  PRIVATE ${MUJOCO_SIMULATE_DIR}/simulate.cc ${MUJOCO_SIMULATE_DIR}/array_safety.h
)
target_include_directories(libsimulate PUBLIC ${MUJOCO_SIMULATE_DIR} ${MUJOCO_INCLUDE_DIR})
target_link_libraries(libsimulate PRIVATE lodepng mujoco::platform_ui_adapter ${MUJOCO_LIB})
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
# MuJoCo has TinyXML2 statically linked, which conflicts with the system TinyXML2
# used by FastDDS. We need to ensure the system TinyXML2 is loaded first.
# Force tinyxml2 to appear first in the DT_NEEDED entries by using LINK_OPTIONS.
# TODO: Remove this after the following is resolved and the next ROS sync
# https://github.com/pal-robotics/mujoco_vendor/issues/2
target_link_options(mujoco_ros2_control PUBLIC
  "LINKER:--push-state,--no-as-needed"
  "$<TARGET_LINKER_FILE:tinyxml2::tinyxml2>"
  "LINKER:--pop-state"
)
target_link_libraries(mujoco_ros2_control
PUBLIC
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mujoco_ros2_control_msgs_TARGETS}
  transmission_interface::transmission_interface
  pluginlib::pluginlib
  ${MUJOCO_LIB}
  control_toolbox::control_toolbox
  Eigen3::Eigen
PRIVATE
  mujoco::libsimulate
  Threads::Threads
  lodepng
  glfw
  OpenGL::EGL
  OpenGL::OpenGL
)
if(OSMESA_FOUND)
  target_link_libraries(mujoco_ros2_control ${OSMESA_LIBRARY})
  target_include_directories(mujoco_ros2_control PRIVATE ${OSMESA_INCLUDE_DIR})
  target_compile_definitions(mujoco_ros2_control PRIVATE OSMESA_AVAILABLE)
endif()
target_include_directories(mujoco_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_SIMULATE_DIR}>
)
# Mark Mujoco source as system to avoid compiler warnings
target_include_directories(mujoco_ros2_control SYSTEM PUBLIC
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${MUJOCO_SIMULATE_DIR}>
)

# Export mujoco alongside the interface to provide access to data.
install(TARGETS mujoco_ros2_control ${MUJOCO_EXPORT_TARGETS}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install runtime deps
install(TARGETS libsimulate lodepng
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

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
  DIRECTORY resources scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

pluginlib_export_plugin_description_file(hardware_interface mujoco_system_interface_plugin.xml)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
