cmake_minimum_required(VERSION 3.22)
project(mujoco_3d_lidar)

find_package(ament_cmake QUIET)
# Link MuJoCo via the vendor package
find_package(mujoco_vendor REQUIRED)

add_library(mujoco_3d_lidar SHARED)
target_sources(mujoco_3d_lidar PRIVATE
  src/register.cpp
  src/3dlidar.cpp
)
target_link_libraries(mujoco_3d_lidar PRIVATE mujoco::mujoco)
target_compile_options(mujoco_3d_lidar PRIVATE ${mujoco_SILENCE_COMPILER_WARNINGS})
target_include_directories(mujoco_3d_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install in the local package, and export
install(TARGETS mujoco_3d_lidar
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS mujoco_3d_lidar
  LIBRARY DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
)
install(DIRECTORY include/ DESTINATION include)

# Duplicate the install to be next to the simulate application so this works there, as well.
# This is best effort because it may not be writeable...
# TODO: Have mujoco_vendor support a writeable plugin directory
find_program(mujoco_SIMULATE NAMES simulate mujoco-simulate)
if(mujoco_SIMULATE)
  cmake_path(SET mujoco_BIN_PATH NORMALIZE ${mujoco_SIMULATE}/..)
  install(CODE "
    file(MAKE_DIRECTORY \"${mujoco_BIN_PATH}/mujoco_plugin\")
    execute_process(
      COMMAND cp -f \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mujoco_plugin/libmujoco_3d_lidar.so\"
                    \"${mujoco_BIN_PATH}/mujoco_plugin/\"
      RESULT_VARIABLE _err)
    if(_err)
      message(WARNING \"Could not install lidar plugin next to simulate! Manually copy it to ${mujoco_BIN_PATH}/mujoco_plugin/\")
    endif()
  ")
else()
  message(WARNING "NO SIMULATE FOUND! "
    "To use the lidar plugin with simulate, manually copy libmujoco_3d_lidar.so "
    "into .../mujoco/bin/mujoco_plugin/")
endif()

# Optionally export
if(ament_cmake_FOUND)
  # Register this package as a mujoco_plugin provider in the ament resource index.
  # The marker file contains the relative path to the plugin directory.
  ament_index_register_resource(mujoco_plugins CONTENT "lib/${PROJECT_NAME}/mujoco_plugin")

  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(mujoco_vendor)
  ament_package()
endif()
