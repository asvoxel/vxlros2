#ifndef VXL_CAMERA__NODE_HELPERS_HPP_
#define VXL_CAMERA__NODE_HELPERS_HPP_

// Helpers shared between VxlCameraNode and VxlCameraLifecycleNode.
//
// Why these live here rather than in each node:
//   declareParameters, the 4 service handlers, and the param-change side-
//   effect blocks (point_cloud / align_depth / filters hot-reload) were near-
//   identical copies in vxl_camera_node.cpp (790 LOC) and
//   vxl_camera_lifecycle_node.cpp (954 LOC). When they drifted, behavior
//   diverged silently between the two ROS variants. Centralising avoids
//   that whole class of bug.
//
// Templated where the only difference is the node concrete type (Node vs
// LifecycleNode); concrete free functions where there's no templating need.

#include "vxl_camera/camera_backend.hpp"
#include "vxl_camera/filter_chain.hpp"
#include "vxl_camera/frame_utils.hpp"
#include "vxl_camera/point_cloud_generator.hpp"
#include "vxl_camera/sensor_options.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vxl_camera_msgs/msg/extrinsics.hpp>
#include <vxl_camera_msgs/msg/metadata.hpp>
#include <vxl_camera_msgs/msg/rgbd.hpp>
#include <vxl_camera_msgs/srv/get_device_info.hpp>
#include <vxl_camera_msgs/srv/get_int32.hpp>
#include <vxl_camera_msgs/srv/set_int32.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace vxl_camera
{

// ─── Parameter declaration ───────────────────────────────────────────────────

// Declare all common parameters on a Node or LifecycleNode. Returns nothing;
// the node reads them after via get_parameter as before. Centralising means
// adding a new parameter requires one edit, not two.
template<typename NodeT>
void declareAllParameters(NodeT & node)
{
  node.declare_parameter("device_serial", "");
  node.declare_parameter("output_mode", "rgbd");
  node.declare_parameter("color.width", 1280);
  node.declare_parameter("color.height", 720);
  node.declare_parameter("color.fps", 30);
  node.declare_parameter("depth.width", 640);
  node.declare_parameter("depth.height", 480);
  node.declare_parameter("depth.fps", 30);
  node.declare_parameter("ir.width", 640);
  node.declare_parameter("ir.height", 480);
  node.declare_parameter("ir.fps", 30);
  node.declare_parameter("point_cloud.enabled", false);
  node.declare_parameter("point_cloud.color", true);
  node.declare_parameter("point_cloud.min_z", 0.0);
  node.declare_parameter("point_cloud.max_z", 0.0);
  node.declare_parameter("point_cloud.decimation", 1);
  node.declare_parameter("point_cloud.organized", true);
  node.declare_parameter("align_depth.enabled", false);
  node.declare_parameter("align_depth.scale", 1.0);

  // Host-side filters
  node.declare_parameter("filters.decimation.enabled", false);
  node.declare_parameter("filters.decimation.scale", 2);
  node.declare_parameter("filters.threshold.enabled", false);
  node.declare_parameter("filters.threshold.min_mm", 100);
  node.declare_parameter("filters.threshold.max_mm", 5000);
  node.declare_parameter("filters.spatial.enabled", false);
  node.declare_parameter("filters.spatial.magnitude", 2);
  node.declare_parameter("filters.spatial.alpha", 0.5);
  node.declare_parameter("filters.spatial.delta", 20.0);
  node.declare_parameter("filters.temporal.enabled", false);
  node.declare_parameter("filters.temporal.alpha", 0.4);
  node.declare_parameter("filters.temporal.delta", 20.0);
  node.declare_parameter("filters.hole_filling.enabled", false);
  node.declare_parameter("filters.hole_filling.mode", 0);

  // Device-side filters (silently no-op on devices that don't expose the option)
  node.declare_parameter("filters.device.denoise.enabled", false);
  node.declare_parameter("filters.device.denoise.level", 2);
  node.declare_parameter("filters.device.median.enabled", false);
  node.declare_parameter("filters.device.median.kernel_size", 3);
  node.declare_parameter("filters.device.outlier_removal.enabled", false);

  node.declare_parameter("sync_mode", "strict");
  node.declare_parameter("frame_queue_size", 4);
  // Inter-stream start delay (ms). 1000 is safe everywhere (matches SDK
  // reference program); drop to 200 on physical Linux for snappier startup;
  // 0 disables (only safe on hardware you've verified doesn't drop frames
  // on back-to-back STREAMONs).
  node.declare_parameter("inter_stream_start_delay_ms", 1000);
  node.declare_parameter("tf_prefix", "");
  node.declare_parameter("publish_tf", true);
}

// Declare ROS2 parameters for SDK options the backend reports as supported.
// Initial value comes from the backend's current setting. Common to both nodes.
template<typename NodeT>
void declareDynamicSensorOptions(NodeT & node, ICameraBackend & backend)
{
  for (const auto & [name, binding] : dynamicOptionTable()) {
    if (!backend.isOptionSupported(binding.sensor, binding.option)) {continue;}
    try {
      auto range = backend.getOptionRange(binding.sensor, binding.option);
      float current = backend.getOption(binding.sensor, binding.option);

      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = std::string(vxl_option_string(binding.option));
      rcl_interfaces::msg::IntegerRange irange;
      irange.from_value = static_cast<int64_t>(range.min);
      irange.to_value = static_cast<int64_t>(range.max);
      irange.step = static_cast<uint64_t>(std::max(1.0f, range.step));
      desc.integer_range.push_back(irange);

      node.template declare_parameter<int>(
        name, static_cast<int>(current), desc);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(node.get_logger(), "Skipping option '%s': %s",
        name.c_str(), e.what());
    }
  }
}

// ─── BackendStreamConfig from parameters ─────────────────────────────────────

template<typename NodeT>
BackendStreamConfig buildBackendStreamConfig(NodeT & node, OutputMode mode)
{
  BackendStreamConfig config;
  config.color_width  = node.get_parameter("color.width").as_int();
  config.color_height = node.get_parameter("color.height").as_int();
  config.color_fps    = node.get_parameter("color.fps").as_int();
  config.depth_width  = node.get_parameter("depth.width").as_int();
  config.depth_height = node.get_parameter("depth.height").as_int();
  config.depth_fps    = node.get_parameter("depth.fps").as_int();
  config.ir_width     = node.get_parameter("ir.width").as_int();
  config.ir_height    = node.get_parameter("ir.height").as_int();
  config.ir_fps       = node.get_parameter("ir.fps").as_int();
  config.sync_mode    = node.get_parameter("sync_mode").as_string();
  config.frame_queue_size = node.get_parameter("frame_queue_size").as_int();
  config.inter_stream_start_delay_ms =
    node.get_parameter("inter_stream_start_delay_ms").as_int();

  switch (mode) {
    case OutputMode::RGBD:
    case OutputMode::RGBDepth:
      config.color_enabled = true; config.depth_enabled = true; break;
    case OutputMode::ColorOnly: config.color_enabled = true; break;
    case OutputMode::DepthOnly: config.depth_enabled = true; break;
    case OutputMode::IR:        config.ir_enabled = true; break;
    case OutputMode::All:
      config.color_enabled = true; config.depth_enabled = true;
      config.ir_enabled = true; break;
  }
  return config;
}

// ─── Free-form service handlers ──────────────────────────────────────────────

// All four return void and write the response in-place; a thrown exception
// inside the backend call is converted to res->success=false + message.

void handleGetDeviceInfo(
  ICameraBackend & backend,
  const vxl_camera_msgs::srv::GetDeviceInfo::Request::SharedPtr & req,
  vxl_camera_msgs::srv::GetDeviceInfo::Response::SharedPtr & res);

void handleGetOption(
  ICameraBackend & backend,
  const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr & req,
  vxl_camera_msgs::srv::GetInt32::Response::SharedPtr & res);

void handleSetOption(
  ICameraBackend & backend,
  const vxl_camera_msgs::srv::SetInt32::Request::SharedPtr & req,
  vxl_camera_msgs::srv::SetInt32::Response::SharedPtr & res);

// Full close → hwReset → re-open sequence. The bare backend.hwReset() left
// the device in a state where the next open would fail (see
// feedback_vxl_device_reset memory). This now does the full recovery so the
// service can be invoked at runtime without a manual restart.
void handleHwReset(
  ICameraBackend & backend,
  const std::string & target_serial,
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  std_srvs::srv::Trigger::Response::SharedPtr & res);

// ─── Message-construction helpers (no node coupling) ─────────────────────────

// Build a sensor_msgs::Image from a BackendFrame. Returns nullptr if the
// format isn't directly publishable (backend should have converted upstream).
//
// Returns UniquePtr (rather than SharedPtr) so callers can `std::move` into
// publish() and unlock intra-process zero-copy when use_intra_process_comms
// is set. The shared-ptr wrapper is added back via toShared() for
// image_transport publish (which still requires by-ref).
sensor_msgs::msg::Image::UniquePtr buildImageMsg(
  const BackendFramePtr & frame,
  const std::string & frame_id);

// Convenience: copy a UniquePtr Image to a new SharedPtr Image (for callers
// that still need shared semantics, e.g. point-cloud generator that consumes
// the same depth twice).
inline sensor_msgs::msg::Image::SharedPtr toShared(
  sensor_msgs::msg::Image::UniquePtr msg)
{
  return sensor_msgs::msg::Image::SharedPtr(msg.release());
}

// Build a Metadata message; returns false if frame is null or invalid.
bool buildMetadataMsg(
  const BackendFramePtr & frame,
  const std::string & frame_id,
  vxl_camera_msgs::msg::Metadata & out);

// Build a RGBD composite. ts_us is taken from the depth frame (canonical
// stamp matches sync_mode tolerance bookkeeping); CameraInfo headers are
// rewritten to match.
void buildRGBDMsg(
  const BackendFramePtr & color,
  const BackendFramePtr & depth,
  const std::string & link_frame_id,
  const std::string & color_frame_id,
  const std::string & depth_frame_id,
  const sensor_msgs::msg::CameraInfo::SharedPtr & color_info,
  const sensor_msgs::msg::CameraInfo::SharedPtr & depth_info,
  vxl_camera_msgs::msg::RGBD & out);

// Build the depth→color extrinsics message (mm → m unit conversion centralised).
// Returns false if the backend can't supply extrinsics for this device.
bool buildExtrinsicsMsg(
  ICameraBackend & backend,
  vxl_camera_msgs::msg::Extrinsics & out);

// ─── Parameter-change side effects ───────────────────────────────────────────

// Pull a fresh PointCloudFilter from the node's parameter values, overlaying
// any pending values from `params` (because get_parameter() inside
// on_set_parameters_callback returns the COMMITTED value, not the proposed one).
template<typename NodeT>
PointCloudFilter readPointCloudFilter(
  NodeT & node, const std::vector<rclcpp::Parameter> & params)
{
  PointCloudFilter f;
  f.min_z_m = static_cast<float>(node.get_parameter("point_cloud.min_z").as_double());
  f.max_z_m = static_cast<float>(node.get_parameter("point_cloud.max_z").as_double());
  f.decimation = node.get_parameter("point_cloud.decimation").as_int();
  f.organized = node.get_parameter("point_cloud.organized").as_bool();
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "point_cloud.min_z") {f.min_z_m = static_cast<float>(p.as_double());}
    else if (n == "point_cloud.max_z") {f.max_z_m = static_cast<float>(p.as_double());}
    else if (n == "point_cloud.decimation") {f.decimation = static_cast<int>(p.as_int());}
    else if (n == "point_cloud.organized") {f.organized = p.as_bool();}
  }
  return f;
}

template<typename NodeT>
std::pair<bool, float> readAlignDepthOverride(
  NodeT & node, const std::vector<rclcpp::Parameter> & params)
{
  bool enabled = node.get_parameter("align_depth.enabled").as_bool();
  float scale = static_cast<float>(node.get_parameter("align_depth.scale").as_double());
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "align_depth.enabled") {enabled = p.as_bool();}
    else if (n == "align_depth.scale") {scale = static_cast<float>(p.as_double());}
  }
  return {enabled, scale};
}

// Apply a parameter batch to the backend: dispatches dynamic SDK-bound options,
// rejects cold parameters, hot-reloads point cloud / align / filter chain.
// Returns the SetParametersResult to surface to rclcpp.
//
// Templated on NodeT so the same code works for Node and LifecycleNode.
template<typename NodeT>
rcl_interfaces::msg::SetParametersResult applyParameterChange(
  NodeT & node,
  ICameraBackend & backend,
  PointCloudGenerator * pc_generator,
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const auto & cold = coldParameters();
  const auto & table = dynamicOptionTable();

  auto get_int_or = [&node](const std::string & n) -> int {
      return node.has_parameter(n) ?
             static_cast<int>(node.get_parameter(n).as_int()) : 0;
    };
  auto dep = checkOptionDependencies(params,
      get_int_or("color.auto_exposure"),
      get_int_or("color.auto_white_balance"),
      get_int_or("depth.auto_exposure"));
  if (!dep.ok) {
    result.successful = false;
    result.reason = dep.reason;
    RCLCPP_WARN(node.get_logger(), "%s", dep.reason.c_str());
    return result;
  }

  for (const auto & param : params) {
    const auto & name = param.get_name();
    if (cold.count(name)) {
      result.successful = false;
      result.reason = "Parameter '" + name +
        "' cannot be changed at runtime; restart / cleanup the node.";
      RCLCPP_WARN(node.get_logger(), "%s", result.reason.c_str());
      return result;
    }
    auto it = table.find(name);
    if (it != table.end()) {
      try {
        if (!backend.isOptionSupported(it->second.sensor, it->second.option)) {
          result.successful = false;
          result.reason = "Option not supported by device: " + name;
          return result;
        }
        backend.setOption(it->second.sensor, it->second.option,
          static_cast<float>(param.as_int()));
        RCLCPP_INFO(node.get_logger(), "Set %s = %ld",
          name.c_str(), param.as_int());
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = std::string("SDK error setting ") + name + ": " + e.what();
        RCLCPP_ERROR(node.get_logger(), "%s", result.reason.c_str());
        return result;
      }
    }
  }

  // Side-effects.
  if (pc_generator &&
    std::any_of(params.begin(), params.end(), [](const rclcpp::Parameter & p) {
      return p.get_name().rfind("point_cloud.", 0) == 0;
    }))
  {
    pc_generator->setFilter(readPointCloudFilter(node, params));
  }

  if (std::any_of(params.begin(), params.end(), [](const rclcpp::Parameter & p) {
    return p.get_name().rfind("align_depth.", 0) == 0;
  })) {
    auto [enabled, scale] = readAlignDepthOverride(node, params);
    backend.setAlignDepthToColor(enabled, scale);
    RCLCPP_INFO(node.get_logger(), "align_depth: enabled=%d scale=%.2f",
      static_cast<int>(enabled), scale);
  }

  if (std::any_of(params.begin(), params.end(), [](const rclcpp::Parameter & p) {
    return p.get_name().rfind(kFilterParamPrefix, 0) == 0;
  })) {
    auto fc = readFilterChainParams(node);
    applyFilterParamOverrides(fc, params);
    backend.setFilterChain(fc);
    RCLCPP_INFO(node.get_logger(), "Filter chain: %s",
      filterChainSummary(fc).c_str());
  }

  return result;
}

}  // namespace vxl_camera

#endif  // VXL_CAMERA__NODE_HELPERS_HPP_
