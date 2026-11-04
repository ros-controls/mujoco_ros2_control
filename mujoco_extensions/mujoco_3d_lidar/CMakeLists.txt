cmake_minimum_required(VERSION 3.22)
project(mujoco_3d_lidar)

find_package(ament_cmake QUIET)
find_package(mujoco_vendor REQUIRED)

add_library(mujoco_3d_lidar SHARED
  src/register.cpp
  src/3dlidar.cpp
)
target_link_libraries(mujoco_3d_lidar PRIVATE mujoco::mujoco)
target_include_directories(mujoco_3d_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS mujoco_3d_lidar
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}/mujoco_plugin
)
install(DIRECTORY include/ DESTINATION include)

if(mujoco_SIMULATE)
  cmake_path(SET mujoco_BIN_PATH NORMALIZE ${mujoco_SIMULATE}/..)
  install(CODE "
    execute_process(
      COMMAND cp -f \"\${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mujoco_plugin/libmujoco_3d_lidar.so\"
                    \"${mujoco_BIN_PATH}/mujoco_plugin/\"
      RESULT_VARIABLE _err)
    if(_err)
      message(WARNING \"Could not copy plugin to ${MUJOCO_PLUGIN_DIR}: \${_err}\")
    endif()
  ")
else()
  message(WARNING "MUJOCO_PLUGIN_DIR not set by mujoco_vendor. "
    "To use the lidar plugin with simulate, manually copy libmujoco_3d_lidar.so "
    "into the mujoco_plugin/ directory next to the simulate binary.")
endif()

if(ament_cmake_FOUND)
  ament_index_register_resource(mujoco_plugins CONTENT "lib/${PROJECT_NAME}/mujoco_plugin")
  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(mujoco_vendor)
  ament_package()
endif()
