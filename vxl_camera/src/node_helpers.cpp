#include "vxl_camera/node_helpers.hpp"

#include <chrono>
#include <cstring>
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
  // Single memcpy from the SDK-owned buffer (or BackendFrame's own vector,
  // for mock / converted frames) into the Image's vector. ROS's transport
  // owns the Image's bytes from here on. This is the only copy on the
  // SDK→ROS publish path; the prior v0.2.0 code was 2 copies (SDK→
  // BackendFrame.data, then BackendFrame.data→Image.data).
  const size_t n = frame->dataSize();
  msg->data.resize(n);
  if (n > 0) {std::memcpy(msg->data.data(), frame->data(), n);}
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

// ─── Diagnostics ─────────────────────────────────────────────────────────────

namespace
{

void appendStreamStatus(
  diagnostic_msgs::msg::DiagnosticArray & arr,
  const std::string & name,
  const StreamDiagCounters & counters,
  rclcpp::Time now,
  bool stream_enabled,
  bool backend_streaming)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;

  DiagnosticStatus s;
  s.name = "vxl_camera/" + name;
  s.hardware_id = name;

  uint64_t total = counters.total_frames.load(std::memory_order_relaxed);
  uint64_t last_ns = counters.last_publish_ns.load(std::memory_order_relaxed);
  double age_s = -1.0;
  if (last_ns != 0) {
    int64_t age_ns = now.nanoseconds() - static_cast<int64_t>(last_ns);
    age_s = age_ns / 1e9;
  }

  if (!stream_enabled) {
    s.level = DiagnosticStatus::OK;
    s.message = "stream disabled";
  } else if (!backend_streaming) {
    s.level = DiagnosticStatus::ERROR;
    s.message = "backend not streaming";
  } else if (last_ns == 0) {
    s.level = DiagnosticStatus::WARN;
    s.message = "no frames yet";
  } else if (age_s < 2.0) {
    s.level = DiagnosticStatus::OK;
    s.message = "streaming";
  } else if (age_s < 10.0) {
    s.level = DiagnosticStatus::WARN;
    s.message = "stalling (no frame in " + std::to_string(static_cast<int>(age_s)) + "s)";
  } else {
    s.level = DiagnosticStatus::ERROR;
    s.message = "stale (no frame in " + std::to_string(static_cast<int>(age_s)) + "s)";
  }

  KeyValue kv;
  kv.key = "total_frames"; kv.value = std::to_string(total);
  s.values.push_back(kv);
  kv.key = "last_frame_age_s";
  kv.value = (age_s < 0) ? std::string("n/a") : std::to_string(age_s);
  s.values.push_back(kv);

  arr.status.push_back(std::move(s));
}

}  // namespace

diagnostic_msgs::msg::DiagnosticArray buildDiagnosticArray(
  const DiagnosticInputs & in,
  rclcpp::Time stamp)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = stamp;

  // Device-level status.
  DiagnosticStatus dev;
  dev.name = "vxl_camera/device";
  dev.hardware_id = "vxl_camera";
  bool backend_streaming = false;
  std::string serial = "(unknown)";
  std::string fw = "(unknown)";
  std::string product = "(unknown)";
  if (in.backend) {
    try {
      backend_streaming = in.backend->isStreaming();
      auto info = in.backend->getDeviceInfo();
      serial = info.serial_number;
      fw = info.fw_version;
      product = info.name;
    } catch (const std::exception &) {
      // Device closed mid-poll — not fatal for diagnostics.
    }
  }
  if (!in.device_present) {
    dev.level = DiagnosticStatus::ERROR;
    dev.message = "device disconnected";
  } else if (!backend_streaming) {
    dev.level = DiagnosticStatus::WARN;
    dev.message = "device idle (not streaming)";
  } else {
    dev.level = DiagnosticStatus::OK;
    dev.message = "streaming";
  }
  KeyValue kv;
  kv.key = "serial"; kv.value = serial; dev.values.push_back(kv);
  kv.key = "firmware"; kv.value = fw; dev.values.push_back(kv);
  kv.key = "product"; kv.value = product; dev.values.push_back(kv);
  kv.key = "output_mode"; kv.value = in.output_mode; dev.values.push_back(kv);
  kv.key = "sync_mode"; kv.value = in.sync_mode; dev.values.push_back(kv);
  arr.status.push_back(std::move(dev));

  // Per-stream status (only if counter pointer was provided — i.e. that
  // stream was enabled in the current output_mode).
  if (in.color) {
    appendStreamStatus(arr, "color", *in.color, stamp, true, backend_streaming);
  }
  if (in.depth) {
    appendStreamStatus(arr, "depth", *in.depth, stamp, true, backend_streaming);
  }
  if (in.ir) {
    appendStreamStatus(arr, "ir", *in.ir, stamp, true, backend_streaming);
  }

  return arr;
}

}  // namespace vxl_camera
