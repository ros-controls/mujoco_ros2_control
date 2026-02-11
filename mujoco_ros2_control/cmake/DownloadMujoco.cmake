include_guard(GLOBAL)

set(MUJOCO_EXPORT_TARGETS "")
set(MUJOCO_INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/mujoco)
set(MUJOCO_INSTALL_SIMULATE_DIR ${CMAKE_INSTALL_PREFIX}/include/mujoco_simulate)

# Fetch the required library from GitHub
include(FetchContent)
set(MUJOCO_VERSION "3.4.0")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
  set(CPU_ARCH "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(CPU_ARCH "aarch64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

message(STATUS "No MuJoCo installation found. Downloading MuJoCo ${MUJOCO_VERSION} for ${CPU_ARCH}.")

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
find_library(_mujoco_lib mujoco HINTS ${MUJOCO_DIR}/lib NO_DEFAULT_PATH)
if(NOT _mujoco_lib)
  message(FATAL_ERROR "Failed to find MuJoCo library in ${MUJOCO_DIR}/lib")
endif()

message(STATUS "MuJoCo downloaded to: ${MUJOCO_DIR}")
set(MUJOCO_LIB_PATH ${_mujoco_lib})
set(MUJOCO_INCLUDE_DIR ${MUJOCO_DIR}/include)
set(MUJOCO_SIMULATE_DIR ${MUJOCO_DIR}/simulate)
set(MUJOCO_FOUND TRUE)
unset(_mujoco_lib CACHE)

# Create wrapper target for proper exports
add_library(mujoco_wrapper INTERFACE)
target_link_libraries(mujoco_wrapper INTERFACE
  $<BUILD_INTERFACE:${MUJOCO_LIB_PATH}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/lib/libmujoco.so>
)
target_include_directories(mujoco_wrapper INTERFACE
  $<BUILD_INTERFACE:${MUJOCO_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:${MUJOCO_INSTALL_INCLUDE_DIR}>
)
set(MUJOCO_LIB mujoco_wrapper)
set(MUJOCO_EXPORT_TARGETS mujoco_wrapper)

install(DIRECTORY ${MUJOCO_DIR}/lib/ DESTINATION lib FILES_MATCHING PATTERN "libmujoco.so*")
install(DIRECTORY ${MUJOCO_INCLUDE_DIR}/ DESTINATION include/mujoco)
install(DIRECTORY ${MUJOCO_SIMULATE_DIR}/ DESTINATION include/mujoco_simulate)
