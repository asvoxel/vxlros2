#include "vxl_camera/vxl_camera_node.hpp"
#include "vxl_camera/sensor_options.hpp"
#include "vxl_camera/frame_utils.hpp"
#include "vxl_camera/filter_chain.hpp"
#include "vxl_camera/node_helpers.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>

namespace vxl_camera
{

VxlCameraNode::VxlCameraNode(const rclcpp::NodeOptions & options)
: VxlCameraNode(options, makeSdkCameraBackend())
{
}

VxlCameraNode::VxlCameraNode(const rclcpp::NodeOptions & options, CameraBackendPtr backend)
: rclcpp::Node("vxl_camera", options),
  backend_(std::move(backend))
{
  if (!backend_) {throw std::runtime_error("CameraBackend must not be null");}
  try {
    declareParameters();
    initDevice();
    declareDynamicSensorOptions(*this, *backend_);
    // Register the on-set callback after all declares to skip initial fires.
    param_cb_handle_ = add_on_set_parameters_callback(
      std::bind(&VxlCameraNode::onParameterChange, this, std::placeholders::_1));
    setupPublishers();
    setupServices();

    if (publish_tf_) {
      publishStaticTFs();
    }

    startStreaming();

    RCLCPP_INFO(get_logger(), "VxlCameraNode started, output_mode: %s",
      get_parameter("output_mode").as_string().c_str());
  } catch (...) {
    // Guarantee cleanup on any exception during construction
    shutdownDevice();
    throw;
  }
}

VxlCameraNode::~VxlCameraNode()
{
  shutdownDevice();
}

void VxlCameraNode::declareParameters()
{
  declareAllParameters(*this);

  // Read post-declare config
  std::string mode_str = get_parameter("output_mode").as_string();
  auto mode = parseOutputMode(mode_str);
  if (!mode) {
    RCLCPP_WARN(get_logger(), "Unknown output_mode '%s', defaulting to 'rgbd'",
      mode_str.c_str());
    output_mode_ = OutputMode::RGBD;
  } else {
    output_mode_ = *mode;
  }
  tf_prefix_ = get_parameter("tf_prefix").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();
}

void VxlCameraNode::initDevice()
{
  std::string serial = get_parameter("device_serial").as_string();
  if (!backend_->open(serial)) {
    throw std::runtime_error(
      serial.empty() ? "No ASVXL camera found" : "Device not found: " + serial);
  }

  auto info = backend_->getDeviceInfo();
  RCLCPP_INFO(get_logger(), "Device: %s (S/N: %s, FW: %s)",
    info.name.c_str(), info.serial_number.c_str(), info.fw_version.c_str());

  if (auto ci = backend_->getIntrinsics(vxl::SensorType::Color)) {
    color_camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(
      buildCameraInfo(*ci, tf_prefix_ + "vxl_color_optical_frame"));
  } else {
    RCLCPP_WARN(get_logger(), "Color intrinsics unavailable");
  }
  if (auto di = backend_->getIntrinsics(vxl::SensorType::Depth)) {
    depth_camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(
      buildCameraInfo(*di, tf_prefix_ + "vxl_depth_optical_frame"));
  } else {
    RCLCPP_WARN(get_logger(), "Depth intrinsics unavailable");
  }
}

void VxlCameraNode::setupPublishers()
{
  auto qos = rclcpp::SensorDataQoS();

  bool need_color = (output_mode_ == OutputMode::RGBD ||
    output_mode_ == OutputMode::RGBDepth ||
    output_mode_ == OutputMode::ColorOnly ||
    output_mode_ == OutputMode::All);
  bool need_depth = (output_mode_ == OutputMode::RGBD ||
    output_mode_ == OutputMode::RGBDepth ||
    output_mode_ == OutputMode::DepthOnly ||
    output_mode_ == OutputMode::All);
  bool need_ir = (output_mode_ == OutputMode::IR ||
    output_mode_ == OutputMode::All);

  if (output_mode_ == OutputMode::RGBD) {
    rgbd_pub_ = create_publisher<vxl_camera_msgs::msg::RGBD>("~/rgbd", qos);
  }

  // Raw Image + CameraInfo publishers (consistent with lifecycle node + enables
  // intra-process zero-copy when publishing via std::move).
  if (need_color && output_mode_ != OutputMode::RGBD) {
    color_pub_ = create_publisher<sensor_msgs::msg::Image>("~/color/image_raw", qos);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/color/camera_info", qos);
  }

  if (need_depth && output_mode_ != OutputMode::RGBD) {
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/depth/image_raw", qos);
    depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/depth/camera_info", qos);
  }

  if (need_ir) {
    ir_pub_ = create_publisher<sensor_msgs::msg::Image>("~/ir/image_raw", qos);
    ir_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/ir/camera_info", qos);
  }

  // Aligned depth: created when both color + depth are flowing. Actual publish
  // is still gated on align_depth.enabled and the backend producing the frame.
  if (need_color && need_depth) {
    aligned_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/aligned_depth_to_color/image_raw", qos);
    aligned_depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/aligned_depth_to_color/camera_info", qos);
  }

  // Per-stream metadata publishers (timestamp / sequence / exposure / gain)
  if (need_color) {
    color_meta_pub_ = create_publisher<vxl_camera_msgs::msg::Metadata>(
      "~/color/metadata", qos);
  }
  if (need_depth) {
    depth_meta_pub_ = create_publisher<vxl_camera_msgs::msg::Metadata>(
      "~/depth/metadata", qos);
  }
  if (need_ir) {
    ir_meta_pub_ = create_publisher<vxl_camera_msgs::msg::Metadata>(
      "~/ir/metadata", qos);
  }

  if (get_parameter("point_cloud.enabled").as_bool() && need_depth) {
    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/depth/points", qos);

    pc_generator_ = std::make_unique<PointCloudGenerator>();
    if (depth_camera_info_) {
      pc_generator_->configure(*depth_camera_info_, 1.0f);
    }

    PointCloudFilter filter;
    filter.min_z_m = static_cast<float>(get_parameter("point_cloud.min_z").as_double());
    filter.max_z_m = static_cast<float>(get_parameter("point_cloud.max_z").as_double());
    filter.decimation = get_parameter("point_cloud.decimation").as_int();
    filter.organized = get_parameter("point_cloud.organized").as_bool();
    pc_generator_->setFilter(filter);

    // Worker thread offloads generate+publish off the SDK polling thread.
    pc_generator_->startAsync(
      [this](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {
        if (pc_pub_) {pc_pub_->publish(std::move(cloud));}
      });
  }

  extrinsics_pub_ = create_publisher<vxl_camera_msgs::msg::Extrinsics>(
    "~/extrinsics/depth_to_color", rclcpp::QoS(1).transient_local());

  // Push initial filter chain to backend (default: all off → no-op).
  backend_->setFilterChain(readFilterChainParams(*this));

  // Push initial depth-to-color alignment config.
  backend_->setAlignDepthToColor(
    get_parameter("align_depth.enabled").as_bool(),
    static_cast<float>(get_parameter("align_depth.scale").as_double()));

  // Publish extrinsics once (latched)
  vxl_camera_msgs::msg::Extrinsics ext_msg;
  if (buildExtrinsicsMsg(*backend_, ext_msg)) {
    extrinsics_pub_->publish(ext_msg);
  } else {
    RCLCPP_WARN(get_logger(), "Extrinsics unavailable");
  }
}

void VxlCameraNode::setupServices()
{
  device_info_srv_ = create_service<vxl_camera_msgs::srv::GetDeviceInfo>(
    "~/get_device_info",
    std::bind(&VxlCameraNode::onGetDeviceInfo, this,
    std::placeholders::_1, std::placeholders::_2));

  get_option_srv_ = create_service<vxl_camera_msgs::srv::GetInt32>(
    "~/get_option",
    std::bind(&VxlCameraNode::onGetOption, this,
    std::placeholders::_1, std::placeholders::_2));

  set_option_srv_ = create_service<vxl_camera_msgs::srv::SetInt32>(
    "~/set_option",
    std::bind(&VxlCameraNode::onSetOption, this,
    std::placeholders::_1, std::placeholders::_2));

  hw_reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/hw_reset",
    std::bind(&VxlCameraNode::onHwReset, this,
    std::placeholders::_1, std::placeholders::_2));
}

void VxlCameraNode::startStreaming()
{
  backend_->startStreaming(
    buildBackendStreamConfig(*this, output_mode_),
    std::bind(&VxlCameraNode::framesetCallback, this, std::placeholders::_1));
}

void VxlCameraNode::shutdownDevice()
{
  if (backend_) {
    backend_->stopStreaming();
    backend_->close();
  }
}

void VxlCameraNode::framesetCallback(BackendFrameSetPtr frameset)
{
  if (!frameset || frameset->empty()) {
    return;
  }

  auto color_frame = frameset->color;
  auto depth_frame = frameset->depth;
  auto ir_frame = frameset->ir;

  switch (output_mode_) {
    case OutputMode::RGBD:
      if (color_frame && depth_frame) {
        publishRGBD(color_frame, depth_frame);
        color_stats_.frames_published++;
        depth_stats_.frames_published++;
      }
      break;
    case OutputMode::RGBDepth:
      if (color_frame) {publishColor(color_frame); color_stats_.frames_published++;}
      if (depth_frame) {publishDepth(depth_frame); depth_stats_.frames_published++;}
      break;
    case OutputMode::ColorOnly:
      if (color_frame) {publishColor(color_frame); color_stats_.frames_published++;}
      break;
    case OutputMode::DepthOnly:
      if (depth_frame) {publishDepth(depth_frame); depth_stats_.frames_published++;}
      break;
    case OutputMode::IR:
      if (ir_frame) {publishIR(ir_frame); ir_stats_.frames_published++;}
      break;
    case OutputMode::All:
      if (color_frame) {publishColor(color_frame); color_stats_.frames_published++;}
      if (depth_frame) {publishDepth(depth_frame); depth_stats_.frames_published++;}
      if (ir_frame) {publishIR(ir_frame); ir_stats_.frames_published++;}
      break;
  }

  // Per-stream metadata
  if (color_frame) {
    publishMetadata(color_meta_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame");
  }
  if (depth_frame) {
    publishMetadata(depth_meta_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame");
  }
  if (ir_frame) {
    publishMetadata(ir_meta_pub_, ir_frame, tf_prefix_ + "vxl_ir_optical_frame");
  }

  // Point cloud (if enabled and depth available)
  if (pc_pub_ && depth_frame) {
    publishPointCloud(depth_frame, color_frame);
  }

  // Aligned depth: publish only when the backend produced one (i.e.,
  // align_depth.enabled is true and color+depth both arrived). camera_info
  // uses the COLOR frame's intrinsics since the aligned depth is in the
  // color view; frame_id is the color optical frame.
  if (frameset->aligned_depth && aligned_depth_pub_) {
    auto img = frameToImageMsg(frameset->aligned_depth,
        tf_prefix_ + "vxl_color_optical_frame");
    if (img) {
      if (aligned_depth_info_pub_ && color_camera_info_) {
        sensor_msgs::msg::CameraInfo info = *color_camera_info_;
        info.header = img->header;
        aligned_depth_info_pub_->publish(info);
      }
      aligned_depth_pub_->publish(std::move(img));
    }
  }
}

void VxlCameraNode::publishMetadata(
  const rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr & pub,
  const BackendFramePtr & frame,
  const std::string & frame_id)
{
  if (!pub) {return;}
  vxl_camera_msgs::msg::Metadata msg;
  if (buildMetadataMsg(frame, frame_id, msg)) {
    pub->publish(msg);
  }
}

void VxlCameraNode::publishRGBD(
  const BackendFramePtr & color,
  const BackendFramePtr & depth)
{
  vxl_camera_msgs::msg::RGBD msg;
  buildRGBDMsg(color, depth,
    tf_prefix_ + "vxl_camera_link",
    tf_prefix_ + "vxl_color_optical_frame",
    tf_prefix_ + "vxl_depth_optical_frame",
    color_camera_info_, depth_camera_info_, msg);
  rgbd_pub_->publish(msg);
}

void VxlCameraNode::publishColor(const BackendFramePtr & frame)
{
  auto img = frameToImageMsg(frame, tf_prefix_ + "vxl_color_optical_frame");
  if (!img || !color_pub_) {return;}
  if (color_info_pub_) {
    sensor_msgs::msg::CameraInfo info = color_camera_info_ ?
      *color_camera_info_ : sensor_msgs::msg::CameraInfo();
    info.header = img->header;
    color_info_pub_->publish(info);
  }
  // std::move enables intra-process zero-copy when use_intra_process_comms
  // is set on the NodeOptions (typical for composable containers).
  color_pub_->publish(std::move(img));
}

void VxlCameraNode::publishDepth(const BackendFramePtr & frame)
{
  auto img = frameToImageMsg(frame, tf_prefix_ + "vxl_depth_optical_frame");
  if (!img || !depth_pub_) {return;}
  if (depth_info_pub_) {
    sensor_msgs::msg::CameraInfo info = depth_camera_info_ ?
      *depth_camera_info_ : sensor_msgs::msg::CameraInfo();
    info.header = img->header;
    depth_info_pub_->publish(info);
  }
  depth_pub_->publish(std::move(img));
}

void VxlCameraNode::publishIR(const BackendFramePtr & frame)
{
  auto img = frameToImageMsg(frame, tf_prefix_ + "vxl_ir_optical_frame");
  if (!img || !ir_pub_) {return;}
  if (ir_info_pub_) {
    sensor_msgs::msg::CameraInfo info;
    info.header = img->header;
    ir_info_pub_->publish(info);
  }
  ir_pub_->publish(std::move(img));
}

void VxlCameraNode::publishPointCloud(
  const BackendFramePtr & depth,
  const BackendFramePtr & color)
{
  if (!pc_generator_) {return;}

  auto depth_img = frameToImageMsg(depth, tf_prefix_ + "vxl_depth_optical_frame");
  if (!depth_img) {return;}

  bool use_color = get_parameter("point_cloud.color").as_bool();
  sensor_msgs::msg::Image::SharedPtr color_img;
  if (use_color && color) {
    auto u = frameToImageMsg(color, tf_prefix_ + "vxl_color_optical_frame");
    if (u) {color_img = toShared(std::move(u));}
  }

  // Drop-old worker queue keeps latency low under sustained load.
  pc_generator_->submit(toShared(std::move(depth_img)), color_img,
    tf_prefix_ + "vxl_depth_optical_frame");
}

sensor_msgs::msg::Image::UniquePtr VxlCameraNode::frameToImageMsg(
  const BackendFramePtr & frame,
  const std::string & frame_id)
{
  auto msg = buildImageMsg(frame, frame_id);
  if (!msg && frame && frame->isValid()) {
    // The SDK backend already handles MJPEG/H264 decoding internally; if we
    // still see an unmappable format here, there's no further conversion path.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported frame format: %d (no further conversion attempted)",
      static_cast<int>(frame->format));
  }
  return msg;
}

void VxlCameraNode::publishStaticTFs()
{
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::string base_frame = tf_prefix_ + "vxl_camera_link";
  auto stamp = now();

  // Optical frames use the convention: Z forward, X right, Y down
  // Camera link uses: X forward, Y left, Z up
  // The rotation from camera_link to optical_frame is: Rz(-90) * Rx(-90)
  tf2::Quaternion optical_rotation;
  optical_rotation.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

  auto make_tf = [&](const std::string & child_frame) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = base_frame;
      tf.child_frame_id = child_frame;
      tf.transform.rotation.x = optical_rotation.x();
      tf.transform.rotation.y = optical_rotation.y();
      tf.transform.rotation.z = optical_rotation.z();
      tf.transform.rotation.w = optical_rotation.w();
      return tf;
    };

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(make_tf(tf_prefix_ + "vxl_color_optical_frame"));
  transforms.push_back(make_tf(tf_prefix_ + "vxl_depth_optical_frame"));
  transforms.push_back(make_tf(tf_prefix_ + "vxl_ir_optical_frame"));

  // Apply extrinsics offset to depth/ir frames if available
  if (auto ext = backend_->getExtrinsics(vxl::SensorType::Color, vxl::SensorType::Depth)) {
    auto & depth_tf = transforms[1];
    depth_tf.transform.translation.x = ext->translation[0] / 1000.0;
    depth_tf.transform.translation.y = ext->translation[1] / 1000.0;
    depth_tf.transform.translation.z = ext->translation[2] / 1000.0;
  }

  tf_static_broadcaster_->sendTransform(transforms);
}

// Service callbacks — delegate to free functions in node_helpers.cpp so the
// non-lifecycle and lifecycle nodes share one implementation.
void VxlCameraNode::onGetDeviceInfo(
  const vxl_camera_msgs::srv::GetDeviceInfo::Request::SharedPtr req,
  vxl_camera_msgs::srv::GetDeviceInfo::Response::SharedPtr res)
{
  handleGetDeviceInfo(*backend_, req, res);
}

void VxlCameraNode::onGetOption(
  const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::GetInt32::Response::SharedPtr res)
{
  handleGetOption(*backend_, req, res);
}

void VxlCameraNode::onSetOption(
  const vxl_camera_msgs::srv::SetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::SetInt32::Response::SharedPtr res)
{
  handleSetOption(*backend_, req, res);
}

void VxlCameraNode::onHwReset(
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  handleHwReset(*backend_, get_parameter("device_serial").as_string(), req, res);
}

rcl_interfaces::msg::SetParametersResult VxlCameraNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  return applyParameterChange(*this, *backend_, pc_generator_.get(), params);
}

}  // namespace vxl_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vxl_camera::VxlCameraNode)
