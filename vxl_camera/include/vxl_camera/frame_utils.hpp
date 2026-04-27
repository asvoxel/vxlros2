#ifndef VXL_CAMERA__FRAME_UTILS_HPP_
#define VXL_CAMERA__FRAME_UTILS_HPP_

#include <sensor_msgs/msg/camera_info.hpp>

#include <vxl.hpp>

#include <optional>
#include <string>
#include <string_view>

// Pure helpers that don't touch ROS Node state — testable without an SDK device.
namespace vxl_camera
{

enum class OutputMode {
  RGBD,        // single synchronized RGBD topic (default)
  RGBDepth,    // separate ~/color and ~/depth topics
  IR,          // debug: IR only
  DepthOnly,   // debug: depth only
  ColorOnly,   // debug: color only
  All,         // debug: every available stream
};

// Parse the user-facing mode string. Returns std::nullopt for unknown values
// so callers decide whether to warn and fall back.
std::optional<OutputMode> parseOutputMode(std::string_view s);

// Inverse mapping — useful for diagnostics output and tests.
std::string outputModeToString(OutputMode mode);

// Map a VxlSense pixel format to a ROS sensor_msgs encoding string.
// Returns std::nullopt for formats that need conversion (MJPEG, H264, NV12, ...).
std::optional<std::string> vxlFormatToRosEncoding(vxl::Format fmt);

// Pure conversion: SDK intrinsics → sensor_msgs/CameraInfo. distortion_model
// is set to "plumb_bob"; D contains [k1,k2,p1,p2,k3]; K and P populated;
// R is identity.
sensor_msgs::msg::CameraInfo buildCameraInfo(
  const vxl::Intrinsics & intrin,
  const std::string & frame_id);

}  // namespace vxl_camera

#endif  // VXL_CAMERA__FRAME_UTILS_HPP_
