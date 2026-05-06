#include "vxl_camera/node_helpers.hpp"

#include <chrono>
#include <thread>

namespace vxl_camera
{

// ─── Service handlers ────────────────────────────────────────────────────────

void handleGetDeviceInfo(
  ICameraBackend & backend,
  const vxl_camera_msgs::srv::GetDeviceInfo::Request::SharedPtr & /*req*/,
  vxl_camera_msgs::srv::GetDeviceInfo::Response::SharedPtr & res)
{
  try {
    auto info = backend.getDeviceInfo();
    res->device_info.name = info.name;
    res->device_info.serial_number = info.serial_number;
    res->device_info.firmware_version = info.fw_version;
    res->device_info.vendor_id = info.vendor_id;
    res->device_info.product_id = info.product_id;
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void handleGetOption(
  ICameraBackend & backend,
  const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr & req,
  vxl_camera_msgs::srv::GetInt32::Response::SharedPtr & res)
{
  try {
    auto sensor_type = parseSensorSelector(req->sensor);
    if (!sensor_type) {
      res->success = false;
      res->message = "Invalid sensor '" + req->sensor + "'; expected color|depth|ir";
      return;
    }
    auto option = static_cast<vxl_option_t>(std::stoi(req->option_name));
    if (!backend.isOptionSupported(*sensor_type, option)) {
      res->success = false;
      res->message = "Option " + req->option_name + " not supported on " + req->sensor;
      return;
    }
    res->value = static_cast<int32_t>(backend.getOption(*sensor_type, option));
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void handleSetOption(
  ICameraBackend & backend,
  const vxl_camera_msgs::srv::SetInt32::Request::SharedPtr & req,
  vxl_camera_msgs::srv::SetInt32::Response::SharedPtr & res)
{
  try {
    auto sensor_type = parseSensorSelector(req->sensor);
    if (!sensor_type) {
      res->success = false;
      res->message = "Invalid sensor '" + req->sensor + "'; expected color|depth|ir";
      return;
    }
    auto option = static_cast<vxl_option_t>(std::stoi(req->option_name));
    if (!backend.isOptionSupported(*sensor_type, option)) {
      res->success = false;
      res->message = "Option " + req->option_name + " not supported on " + req->sensor;
      return;
    }
    backend.setOption(*sensor_type, option, static_cast<float>(req->value));
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void handleHwReset(
  ICameraBackend & backend,
  const std::string & target_serial,
  const std_srvs::srv::Trigger::Request::SharedPtr & /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  // Full recovery: stop streaming → close → hwReset → re-open. Bare hwReset
  // would leave the device in a state where the next open fails (per
  // feedback memory: VXL615 close/reset must be paired with USB reset before
  // next open). The caller doesn't know the device is now closed; doing the
  // re-open inside the service makes the behavior obvious from the client's
  // perspective ("did it return success? then I can keep using the topics").
  try {
    backend.stopStreaming();
    backend.close();
    backend.hwReset();
    // Brief settle so kernel uvcvideo re-enumerates the device cleanly.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!backend.open(target_serial)) {
      res->success = false;
      res->message = "Reset succeeded but reopen failed (serial=" +
        target_serial + ")";
      return;
    }
    res->success = true;
    res->message =
      "Hardware reset + reopen succeeded; clients should reconfigure if needed";
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

// ─── Message construction ────────────────────────────────────────────────────

sensor_msgs::msg::Image::UniquePtr buildImageMsg(
  const BackendFramePtr & frame,
  const std::string & frame_id)
{
  if (!frame || !frame->isValid()) {return nullptr;}
  auto encoding = vxlFormatToRosEncoding(frame->format);
  if (!encoding) {return nullptr;}

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = frame->timestamp_us != 0 ?
    toRosTimeFromHardware(frame->timestamp_us) :
    rclcpp::Time(0, 0, RCL_ROS_TIME);
  msg->header.frame_id = frame_id;
  msg->width = frame->width;
  msg->height = frame->height;
  msg->step = frame->stride;
  msg->is_bigendian = false;
  msg->encoding = *encoding;
  msg->data = frame->data;
  return msg;
}

bool buildMetadataMsg(
  const BackendFramePtr & frame,
  const std::string & frame_id,
  vxl_camera_msgs::msg::Metadata & out)
{
  if (!frame || !frame->isValid()) {return false;}
  out.header.stamp = frame->timestamp_us != 0 ?
    toRosTimeFromHardware(frame->timestamp_us) :
    rclcpp::Time(0, 0, RCL_ROS_TIME);
  out.header.frame_id = frame_id;
  out.timestamp_us = frame->timestamp_us;
  out.frame_number = frame->sequence;
  out.exposure_us = frame->exposure_us;
  out.gain = frame->gain;
  return true;
}

void buildRGBDMsg(
  const BackendFramePtr & color,
  const BackendFramePtr & depth,
  const std::string & link_frame_id,
  const std::string & color_frame_id,
  const std::string & depth_frame_id,
  const sensor_msgs::msg::CameraInfo::SharedPtr & color_info,
  const sensor_msgs::msg::CameraInfo::SharedPtr & depth_info,
  vxl_camera_msgs::msg::RGBD & out)
{
  uint64_t ts_us = depth ? depth->timestamp_us :
    (color ? color->timestamp_us : 0);
  out.header.stamp = ts_us != 0 ?
    toRosTimeFromHardware(ts_us) : rclcpp::Time(0, 0, RCL_ROS_TIME);
  out.header.frame_id = link_frame_id;

  auto rgb_img = buildImageMsg(color, color_frame_id);
  auto depth_img = buildImageMsg(depth, depth_frame_id);
  if (rgb_img) {out.rgb = *rgb_img;}
  if (depth_img) {out.depth = *depth_img;}
  if (color_info) {
    out.rgb_camera_info = *color_info;
    out.rgb_camera_info.header.stamp = out.header.stamp;
  }
  if (depth_info) {
    out.depth_camera_info = *depth_info;
    out.depth_camera_info.header.stamp = out.header.stamp;
  }
}

bool buildExtrinsicsMsg(
  ICameraBackend & backend,
  vxl_camera_msgs::msg::Extrinsics & out)
{
  auto ext = backend.getExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color);
  if (!ext) {return false;}
  for (int i = 0; i < 9; i++) {out.rotation[i] = static_cast<double>(ext->rotation[i]);}
  for (int i = 0; i < 3; i++) {
    out.translation[i] = mmToMeters(static_cast<double>(ext->translation[i]));
  }
  return true;
}

}  // namespace vxl_camera
