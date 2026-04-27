#include "vxl_camera/frame_utils.hpp"

namespace vxl_camera
{

std::optional<OutputMode> parseOutputMode(std::string_view s)
{
  if (s == "rgbd") {return OutputMode::RGBD;}
  if (s == "rgb+depth") {return OutputMode::RGBDepth;}
  if (s == "ir") {return OutputMode::IR;}
  if (s == "depth_only") {return OutputMode::DepthOnly;}
  if (s == "color_only") {return OutputMode::ColorOnly;}
  if (s == "all") {return OutputMode::All;}
  return std::nullopt;
}

std::string outputModeToString(OutputMode mode)
{
  switch (mode) {
    case OutputMode::RGBD:      return "rgbd";
    case OutputMode::RGBDepth:  return "rgb+depth";
    case OutputMode::IR:        return "ir";
    case OutputMode::DepthOnly: return "depth_only";
    case OutputMode::ColorOnly: return "color_only";
    case OutputMode::All:       return "all";
  }
  return "rgbd";
}

std::optional<std::string> vxlFormatToRosEncoding(vxl::Format fmt)
{
  switch (fmt) {
    case vxl::Format::BGR:    return std::string("bgr8");
    case vxl::Format::RGB:    return std::string("rgb8");
    case vxl::Format::Z16:    return std::string("16UC1");
    case vxl::Format::Gray8:  return std::string("mono8");
    case vxl::Format::Gray16: return std::string("mono16");
    default:
      // YUYV / UYVY / MJPEG / H264 / NV12 — caller should convert via SDK first.
      return std::nullopt;
  }
}

sensor_msgs::msg::CameraInfo buildCameraInfo(
  const vxl::Intrinsics & intrin,
  const std::string & frame_id)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = intrin.width;
  info.height = intrin.height;
  info.distortion_model = "plumb_bob";
  info.d.resize(5);
  for (int i = 0; i < 5; i++) {info.d[i] = intrin.coeffs[i];}
  info.k = {
    static_cast<double>(intrin.fx), 0.0, static_cast<double>(intrin.cx),
    0.0, static_cast<double>(intrin.fy), static_cast<double>(intrin.cy),
    0.0, 0.0, 1.0,
  };
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = {
    static_cast<double>(intrin.fx), 0.0, static_cast<double>(intrin.cx), 0.0,
    0.0, static_cast<double>(intrin.fy), static_cast<double>(intrin.cy), 0.0,
    0.0, 0.0, 1.0, 0.0,
  };
  return info;
}

}  // namespace vxl_camera
