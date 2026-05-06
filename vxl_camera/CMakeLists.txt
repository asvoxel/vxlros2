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
# image_transport / camera_info_manager dropped: both nodes now use raw
# Publisher<Image> + Publisher<CameraInfo> for two reasons —
#   1. image_transport doesn't support LifecycleNode in Humble; the lifecycle
#      node had to use raw publishers, leaving non-lifecycle on a different
#      topology (extra compressed/theora siblings) — confusing for users.
#   2. image_transport::CameraPublisher::publish takes (const Image&, …);
#      no UniquePtr overload, blocking intra-process zero-copy.
# Users who need compressed transport can run a republisher node downstream.
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(vxl_camera_msgs REQUIRED)

# ─── ASVXL SDK ───────────────────────────────────────────────────────────────
#
# SDK discovery priority:
#   1. cmake -DVXL_SDK_DIR=/path/to/sdk   (CI / custom builds)
#   2. VXL_SDK_DIR environment variable    (developer override)
#   3. ../../vxlsdk/sdk/current/          (sibling vxlsdk project build output)
#   4. ../vxlsdk/                         (in-repo placeholder for release tarball)
#

if(NOT DEFINED VXL_SDK_DIR)
  if(DEFINED ENV{VXL_SDK_DIR})
    set(VXL_SDK_DIR $ENV{VXL_SDK_DIR})
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../vxlsdk/sdk/current/include/vxl.hpp")
    set(VXL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vxlsdk/sdk/current")
  else()
    set(VXL_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../vxlsdk")
  endif()
endif()

message(STATUS "VXL SDK dir: ${VXL_SDK_DIR}")

# Validate SDK — must contain both C and C++ API headers
if(NOT EXISTS "${VXL_SDK_DIR}/include/vxl.hpp")
  message(FATAL_ERROR
    "ASVXL SDK not found at: ${VXL_SDK_DIR}\n"
    "Provide the SDK in one of these ways:\n"
    "  1. Build the sibling vxlsdk project:\n"
    "       cd ../../vxlsdk && ./scripts/release-{linux,macos}.sh\n"
    "     Output is symlinked at vxlsdk/sdk/current/ and picked up automatically.\n"
    "  2. Extract a release tarball into ${CMAKE_CURRENT_SOURCE_DIR}/../vxlsdk/\n"
    "       (download from https://github.com/asvoxel/vxlsdk-rel/releases)\n"
    "  3. Pass an explicit path: -DVXL_SDK_DIR=/your/path\n"
    "     (or export VXL_SDK_DIR=/your/path)")
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
  message(FATAL_ERROR "ASVXL library not found in ${VXL_LIB_DIR}")
endif()

# libasvxl.a is a static archive that wraps libuvc + libusb-1.0 calls but does
# NOT include their object code. Consumers must link them transitively.
set(VXL_TRANSITIVE_LIBS "")

if(VXL_PLATFORM STREQUAL "linux")
  # libuvc: prefer the bundled static archive shipped alongside libasvxl.a
  # (built from vxlsdk/3rds/libuvc/build); fall back to system libuvc.
  find_library(VXL_LIBUVC NAMES uvc PATHS ${VXL_LIB_DIR} NO_DEFAULT_PATH)
  if(NOT VXL_LIBUVC)
    find_library(VXL_LIBUVC NAMES uvc)
  endif()
  if(NOT VXL_LIBUVC)
    message(FATAL_ERROR
      "libuvc not found. The SDK ships libuvc.a in lib/linux/ when built from\n"
      "  source via vxlsdk's release scripts. Either install libuvc-dev system-wide\n"
      "  or rebuild the SDK and copy 3rds/libuvc/build/libuvc.a into\n"
      "  ${VXL_LIB_DIR}/libuvc.a")
  endif()
  list(APPEND VXL_TRANSITIVE_LIBS ${VXL_LIBUVC})

  # libusb-1.0 is always a system dep on Linux.
  find_library(VXL_LIBUSB NAMES usb-1.0)
  if(NOT VXL_LIBUSB)
    message(FATAL_ERROR "libusb-1.0 not found. apt install libusb-1.0-0-dev")
  endif()
  list(APPEND VXL_TRANSITIVE_LIBS ${VXL_LIBUSB} pthread)

  # libjpeg for MJPEG decoding (libuvc dep).
  find_library(VXL_LIBJPEG NAMES jpeg)
  if(NOT VXL_LIBJPEG)
    message(FATAL_ERROR "libjpeg not found. apt install libjpeg-dev")
  endif()
  list(APPEND VXL_TRANSITIVE_LIBS ${VXL_LIBJPEG})
endif()

# Create proper imported target for IDE support and transitive deps
add_library(vxlsdk::asvxl STATIC IMPORTED)
set_target_properties(vxlsdk::asvxl PROPERTIES
  IMPORTED_LOCATION "${VXL_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${VXL_INCLUDE_DIR}"
  INTERFACE_LINK_LIBRARIES "${VXL_TRANSITIVE_LIBS}"
)

message(STATUS "VXL SDK version: ${VXL_SDK_DIR}")
message(STATUS "VXL library:     ${VXL_LIBRARY}")
message(STATUS "VXL transitive:  ${VXL_TRANSITIVE_LIBS}")

# ─── Component library ───────────────────────────────────────────────────────

set(NODE_SOURCES
  src/vxl_camera_node.cpp
  src/vxl_camera_lifecycle_node.cpp
  src/sdk_camera_backend.cpp
  src/sensor_options.cpp
  src/frame_utils.cpp
  src/filter_chain.cpp
  src/point_cloud_generator.cpp
  src/node_helpers.cpp
)

add_library(vxl_camera_component SHARED ${NODE_SOURCES})

target_include_directories(vxl_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# ASVXL headers are provided via vxlsdk::asvxl imported target

target_link_libraries(vxl_camera_component vxlsdk::asvxl)

ament_target_dependencies(vxl_camera_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  std_srvs
  std_msgs
  tf2_ros
  tf2
  geometry_msgs
  vxl_camera_msgs
)

# Register both components for use in ComposableNodeContainers.
# Only the non-lifecycle component gets a generic executable; the lifecycle
# component has its own thin executable below that auto-triggers
# CONFIGURE → ACTIVATE so `ros2 run` users get the expected behavior.
rclcpp_components_register_node(vxl_camera_component
  PLUGIN "vxl_camera::VxlCameraNode"
  EXECUTABLE vxl_camera_node
)
rclcpp_components_register_nodes(vxl_camera_component
  "vxl_camera::VxlCameraLifecycleNode")

# ─── Lifecycle standalone (auto-activates by default) ────────────────────────

add_executable(vxl_camera_lifecycle_node src/main_lifecycle.cpp)
target_link_libraries(vxl_camera_lifecycle_node vxl_camera_component)
ament_target_dependencies(vxl_camera_lifecycle_node
  rclcpp rclcpp_lifecycle lifecycle_msgs)

# ─── Install ─────────────────────────────────────────────────────────────────

install(TARGETS vxl_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS vxl_camera_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Co-install libuvc shared lib so consumers find it via vxl_camera's install/lib
# (ROS2 setup.bash adds that to LD_LIBRARY_PATH; binary RUNPATH alone is bypassed
# by ROS2's LD_LIBRARY_PATH when transitively loaded through component .so).
if(VXL_PLATFORM STREQUAL "linux")
  file(GLOB VXL_LIBUVC_FILES "${VXL_LIB_DIR}/libuvc.so*")
  if(VXL_LIBUVC_FILES)
    install(FILES ${VXL_LIBUVC_FILES} DESTINATION lib)
  endif()
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ─── Tests ───────────────────────────────────────────────────────────────────

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # Skip cpplint/uncrustify — they conflict with our 2-space + Allman-for-function style.
  # Keep flake8, xmllint, and copyright checks.
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_output_mode test/test_output_mode.cpp)
  target_link_libraries(test_output_mode vxl_camera_component)
  ament_target_dependencies(test_output_mode
    rclcpp sensor_msgs vxl_camera_msgs)

  ament_add_gtest(test_point_cloud test/test_point_cloud.cpp)
  target_link_libraries(test_point_cloud vxl_camera_component)
  ament_target_dependencies(test_point_cloud rclcpp sensor_msgs)

  ament_add_gtest(test_frame_utils test/test_frame_utils.cpp)
  target_link_libraries(test_frame_utils vxl_camera_component)
  ament_target_dependencies(test_frame_utils sensor_msgs)

  ament_add_gtest(test_sensor_options test/test_sensor_options.cpp)
  target_link_libraries(test_sensor_options vxl_camera_component)
  ament_target_dependencies(test_sensor_options rclcpp)

  ament_add_gtest(test_node_with_mock test/test_node_with_mock.cpp)
  target_link_libraries(test_node_with_mock vxl_camera_component)
  ament_target_dependencies(test_node_with_mock
    rclcpp std_srvs vxl_camera_msgs)

  ament_add_gtest(test_lifecycle_with_mock test/test_lifecycle_with_mock.cpp)
  target_link_libraries(test_lifecycle_with_mock vxl_camera_component)
  ament_target_dependencies(test_lifecycle_with_mock
    rclcpp rclcpp_lifecycle lifecycle_msgs std_srvs vxl_camera_msgs)

  ament_add_gtest(test_sdk_camera_backend test/test_sdk_camera_backend.cpp)
  target_link_libraries(test_sdk_camera_backend vxl_camera_component)

  ament_add_gtest(test_filter_chain test/test_filter_chain.cpp)
  target_link_libraries(test_filter_chain vxl_camera_component)
  ament_target_dependencies(test_filter_chain rclcpp)

  # ─── Mock-driven launch test for lifecycle node ─────────────────────────
  # Builds a test-only executable that wires VxlCameraLifecycleNode to a
  # MockCameraBackend + a synthetic frame source. Used by test_lifecycle_launch.py
  # to exercise the full ROS2 launch path end-to-end without hardware.
  add_executable(vxl_camera_lifecycle_mock test/mock_lifecycle_main.cpp)
  target_link_libraries(vxl_camera_lifecycle_mock vxl_camera_component)
  ament_target_dependencies(vxl_camera_lifecycle_mock
    rclcpp rclcpp_lifecycle lifecycle_msgs)
  install(TARGETS vxl_camera_lifecycle_mock
    DESTINATION lib/${PROJECT_NAME})

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_lifecycle_launch.py
    TIMEOUT 60
    TARGET test_lifecycle_launch)
endif()

ament_package()
