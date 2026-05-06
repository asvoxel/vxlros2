#ifndef VXL_CAMERA__FRAME_UTILS_HPP_
#define VXL_CAMERA__FRAME_UTILS_HPP_

#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <vxl.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

// Pure helpers that don't touch ROS Node state — testable without an SDK device.
namespace vxl_camera
{

// Convert an SDK hardware timestamp (microseconds since device epoch) to an
// rclcpp::Time. Uses RCL_ROS_TIME so downstream TF / sync / sensor fusion
// can mix this with `now()` from other nodes without clock-source clashes.
//
// IMPORTANT — timestamp semantics, single device:
//   * MONOTONIC. The SDK guarantees the value strictly increases per
//     successful frame within one open() session.
//   * SUB-FRAME PRECISION. Captures the exposure midpoint to ~10us
//     accuracy on VXL6X5 / VXL435.
//   * NOT WALL-CLOCK ALIGNED. The "device epoch" is whatever the firmware
//     latched at power-up — typically nanoseconds since SoC boot. TF
//     lookups WITHIN a single device's frames work; comparing to system
//     time / GPS / NTP does NOT without an external offset.
//
// IMPORTANT — multi-device / multi-camera:
//   * NOT COMPARABLE across devices. Two cameras' timestamps live in
//     different epochs. For multi-camera fusion the user MUST establish
//     a per-device offset (e.g. `t_corrected = t_device + offset`) by
//     observing one synchronization event per pair, OR drive both
//     cameras off a hardware sync trigger and treat the trigger pulse
//     as t=0 in user code.
//
// IMPORTANT — bag recording:
//   * `ros2 bag record` time-aligns by header.stamp. With device-epoch
//     stamps, recordings will look "shifted" relative to wall clock —
//     this is correct and expected. To force wall-clock recording set
//     ROS parameter `timestamp_source = ros_time` (TODO future).
inline rclcpp::Time toRosTimeFromHardware(uint64_t timestamp_us)
{
  // rclcpp::Time(int64 nanoseconds, clock_type)
  return rclcpp::Time(
    static_cast<int64_t>(timestamp_us) * 1000LL, RCL_ROS_TIME);
}

// Translation unit conversion: SDK extrinsics report mm; ROS TF / camera_info
// expect meters. Centralised so future device families can override per-SKU.
constexpr double kMillimetersPerMeter = 1000.0;
inline double mmToMeters(double mm) {return mm / kMillimetersPerMeter;}

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

// Map an ASVXL pixel format to a ROS sensor_msgs encoding string.
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
