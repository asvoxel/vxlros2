cmake_minimum_required(VERSION 3.8)
project(vxl_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Dependencies ────────────────────────────────────────────────────────────

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vxl_camera_msgs REQUIRED)

# ─── VxlSense SDK ────────────────────────────────────────────────────────────
#
# SDK discovery priority:
#   1. cmake -DVXL_SDK_DIR=/path/to/sdk   (CI / custom builds)
#   2. VXL_SDK_DIR environment variable    (developer override)
#   3. ../vxlsense-sdk/                   (bundled in VxlROS2 repo, default)
#

if(NOT DEFINED VXL_SDK_DIR)
  if(DEFINED ENV{VXL_SDK_DIR})
    set(VXL_SDK_DIR $ENV{VXL_SDK_DIR})
  else()
    set(VXL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../vxlsense-sdk")
  endif()
endif()

message(STATUS "VXL SDK dir: ${VXL_SDK_DIR}")

# Validate SDK — must contain both C and C++ API headers
if(NOT EXISTS "${VXL_SDK_DIR}/include/vxl.hpp")
  message(FATAL_ERROR
    "VxlSense SDK not found at: ${VXL_SDK_DIR}\n"
    "Please install the SDK:\n"
    "  1. Download from https://github.com/asvoxel/VxlSense/releases\n"
    "  2. Extract to ${CMAKE_CURRENT_SOURCE_DIR}/../vxlsense-sdk/\n"
    "  Or set -DVXL_SDK_DIR=/your/path")
endif()

set(VXL_INCLUDE_DIR "${VXL_SDK_DIR}/include")

# Detect platform
if(APPLE)
  set(VXL_PLATFORM "darwin")
elseif(WIN32)
  set(VXL_PLATFORM "windows")
else()
  set(VXL_PLATFORM "linux")
endif()

set(VXL_LIB_DIR "${VXL_SDK_DIR}/lib/${VXL_PLATFORM}")
find_library(VXL_LIBRARY NAMES asvxl vxl PATHS ${VXL_LIB_DIR} NO_DEFAULT_PATH)

if(NOT VXL_LIBRARY)
  message(FATAL_ERROR "VxlSense library not found in ${VXL_LIB_DIR}")
endif()

# Create proper imported target for IDE support and transitive deps
add_library(VxlSense::asvxl STATIC IMPORTED)
set_target_properties(VxlSense::asvxl PROPERTIES
  IMPORTED_LOCATION "${VXL_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${VXL_INCLUDE_DIR}"
)

message(STATUS "VXL SDK version: ${VXL_SDK_DIR}")
message(STATUS "VXL library:     ${VXL_LIBRARY}")

# ─── Component library ───────────────────────────────────────────────────────

set(NODE_SOURCES
  src/vxl_camera_node.cpp
  src/stream_manager.cpp
  src/point_cloud_generator.cpp
)

add_library(vxl_camera_component SHARED ${NODE_SOURCES})

target_include_directories(vxl_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# VxlSense headers are provided via VxlSense::asvxl imported target

target_link_libraries(vxl_camera_component VxlSense::asvxl)

ament_target_dependencies(vxl_camera_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  image_transport
  camera_info_manager
  cv_bridge
  tf2_ros
  tf2
  geometry_msgs
  vxl_camera_msgs
)

rclcpp_components_register_node(vxl_camera_component
  PLUGIN "vxl_camera::VxlCameraNode"
  EXECUTABLE vxl_camera_node
)

# ─── Standalone executable ───────────────────────────────────────────────────

add_executable(vxl_camera_standalone src/main.cpp)
target_link_libraries(vxl_camera_standalone vxl_camera_component)
ament_target_dependencies(vxl_camera_standalone rclcpp)

# ─── Install ─────────────────────────────────────────────────────────────────

install(TARGETS vxl_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS vxl_camera_standalone
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ─── Tests ───────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_output_mode test/test_output_mode.cpp)
  target_link_libraries(test_output_mode vxl_camera_component)
  ament_target_dependencies(test_output_mode rclcpp sensor_msgs vxl_camera_msgs)

  ament_add_gtest(test_point_cloud test/test_point_cloud.cpp)
  target_link_libraries(test_point_cloud vxl_camera_component)
  ament_target_dependencies(test_point_cloud rclcpp sensor_msgs)
endif()

ament_package()
