#include "vxl_camera/vxl_camera_node.hpp"
#include "vxl_camera/sensor_options.hpp"
#include "vxl_camera/frame_utils.hpp"
#include "vxl_camera/filter_chain.hpp"

#include <cv_bridge/cv_bridge.h>
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
    declareDynamicOptions();
    setupPublishers();
    setupServices();
    setupDiagnostics();

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
  // Device
  declare_parameter("device_serial", "");
  declare_parameter("output_mode", "rgbd");

  // Color
  declare_parameter("color.width", 1280);
  declare_parameter("color.height", 720);
  declare_parameter("color.fps", 30);

  // Depth
  declare_parameter("depth.width", 640);
  declare_parameter("depth.height", 480);
  declare_parameter("depth.fps", 30);

  // IR (debug only)
  declare_parameter("ir.width", 640);
  declare_parameter("ir.height", 480);
  declare_parameter("ir.fps", 30);

  // Point cloud
  declare_parameter("point_cloud.enabled", false);
  declare_parameter("point_cloud.color", true);
  declare_parameter("point_cloud.min_z", 0.0);    // meters; 0 = no min limit
  declare_parameter("point_cloud.max_z", 0.0);    // meters; 0 = no max limit
  declare_parameter("point_cloud.decimation", 1);  // pixel skip; 1 = every pixel
  declare_parameter("point_cloud.organized", true);

  // Depth post-processing filters (host-side, applied in SDK polling thread
  // before frames are delivered). Default: all off — backwards compatible.
  declare_parameter("filters.decimation.enabled", false);
  declare_parameter("filters.decimation.scale", 2);          // 2 / 4 / 8
  declare_parameter("filters.threshold.enabled", false);
  declare_parameter("filters.threshold.min_mm", 100);
  declare_parameter("filters.threshold.max_mm", 5000);
  declare_parameter("filters.spatial.enabled", false);
  declare_parameter("filters.spatial.magnitude", 2);          // 1..5
  declare_parameter("filters.spatial.alpha", 0.5);            // 0.25..1.0
  declare_parameter("filters.spatial.delta", 20.0);           // 1..50
  declare_parameter("filters.temporal.enabled", false);
  declare_parameter("filters.temporal.alpha", 0.4);           // 0.0..1.0
  declare_parameter("filters.temporal.delta", 20.0);          // 1..100
  declare_parameter("filters.hole_filling.enabled", false);
  declare_parameter("filters.hole_filling.mode", 0);          // 0/1/2

  // Sync
  declare_parameter("sync_mode", "strict");
  declare_parameter("frame_queue_size", 4);

  // TF
  declare_parameter("tf_prefix", "");
  declare_parameter("publish_tf", true);

  // Read config
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

void VxlCameraNode::declareDynamicOptions()
{
  // Declare ROS2 parameters for SDK options that the backend reports as supported.
  // Initial value reads the live setting from the backend.
  for (const auto & [name, binding] : dynamicOptionTable()) {
    if (!backend_->isOptionSupported(binding.sensor, binding.option)) {continue;}
    try {
      auto range = backend_->getOptionRange(binding.sensor, binding.option);
      float current = backend_->getOption(binding.sensor, binding.option);

      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = std::string(vxl_option_string(binding.option));
      rcl_interfaces::msg::IntegerRange irange;
      irange.from_value = static_cast<int64_t>(range.min);
      irange.to_value = static_cast<int64_t>(range.max);
      irange.step = static_cast<uint64_t>(std::max(1.0f, range.step));
      desc.integer_range.push_back(irange);

      declare_parameter<int>(name, static_cast<int>(current), desc);
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "Skipping option '%s': %s", name.c_str(), e.what());
    }
  }

  // Register the on-set callback after declarations to avoid firing for initial values.
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&VxlCameraNode::onParameterChange, this, std::placeholders::_1));
}

void VxlCameraNode::initDevice()
{
  std::string serial = get_parameter("device_serial").as_string();
  if (!backend_->open(serial)) {
    throw std::runtime_error(
      serial.empty() ? "No VxlSense device found" : "Device not found: " + serial);
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

  if (need_color && output_mode_ != OutputMode::RGBD) {
    color_pub_ = image_transport::create_camera_publisher(
      this, "~/color/image_raw", qos.get_rmw_qos_profile());
  }

  if (need_depth && output_mode_ != OutputMode::RGBD) {
    depth_pub_ = image_transport::create_camera_publisher(
      this, "~/depth/image_raw", qos.get_rmw_qos_profile());
  }

  if (need_ir) {
    ir_pub_ = image_transport::create_camera_publisher(
      this, "~/ir/image_raw", qos.get_rmw_qos_profile());
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

  // Publish extrinsics once (latched)
  if (auto ext = backend_->getExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color)) {
    auto msg = vxl_camera_msgs::msg::Extrinsics();
    for (int i = 0; i < 9; i++) {msg.rotation[i] = static_cast<double>(ext->rotation[i]);}
    for (int i = 0; i < 3; i++) {
      msg.translation[i] = static_cast<double>(ext->translation[i]) / 1000.0;  // mm→m
    }
    extrinsics_pub_->publish(msg);
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

void VxlCameraNode::setupDiagnostics()
{
  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);

  std::string hw_id = "vxl_camera";
  try {
    auto info = backend_->getDeviceInfo();
    hw_id = info.name + ":" + info.serial_number;
  } catch (const std::exception &) {}
  diag_updater_->setHardwareID(hw_id);

  diag_updater_->add("vxl_camera",
    std::bind(&VxlCameraNode::diagnosticsCallback, this, std::placeholders::_1));

  // Trigger updates at 1 Hz; the wrapper handles the actual rate calculations.
  auto now_tp = std::chrono::steady_clock::now();
  color_stats_.last_sample_time = now_tp;
  depth_stats_.last_sample_time = now_tp;
  ir_stats_.last_sample_time = now_tp;

  diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    diag_updater_->force_update();
  });
}

void VxlCameraNode::diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  std::lock_guard<std::mutex> lock(stats_mutex_);
  auto now_tp = std::chrono::steady_clock::now();

  auto report = [&](const char * name, StreamStats & s, int expected_fps) {
    uint64_t cur = s.frames_published.load();
    uint64_t prev = s.last_sample_count.load();
    auto elapsed_s = std::chrono::duration<double>(now_tp - s.last_sample_time).count();
    double fps = (elapsed_s > 0.001) ? (cur - prev) / elapsed_s : 0.0;
    s.last_sample_count.store(cur);
    s.last_sample_time = now_tp;

    stat.add(std::string(name) + ".frames_published", cur);
    stat.add(std::string(name) + ".fps", fps);
    stat.add(std::string(name) + ".expected_fps", expected_fps);

    // Flag if the actual rate falls below 50% of the configured rate while streaming.
    // (Skip the warning during the first sample window where prev == 0.)
    if (expected_fps > 0 && prev > 0 && fps > 0 && fps < expected_fps * 0.5) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::string(name) + " fps below 50% of configured rate");
    }
  };

  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");

  if (color_meta_pub_) {
    report("color", color_stats_, get_parameter("color.fps").as_int());
  }
  if (depth_meta_pub_) {
    report("depth", depth_stats_, get_parameter("depth.fps").as_int());
  }
  if (ir_meta_pub_) {
    report("ir", ir_stats_, get_parameter("ir.fps").as_int());
  }

  // Device info
  try {
    auto info = backend_->getDeviceInfo();
    stat.add("device.name", info.name);
    stat.add("device.serial", info.serial_number);
    stat.add("device.firmware", info.fw_version);
    stat.add("device.connected", backend_->isOpen());
    if (!backend_->isOpen()) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Device disconnected");
    }
  } catch (const std::exception & e) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      std::string("Device query failed: ") + e.what());
  }

  stat.add("output_mode", get_parameter("output_mode").as_string());
}

void VxlCameraNode::startStreaming()
{
  BackendStreamConfig config;
  config.color_width = get_parameter("color.width").as_int();
  config.color_height = get_parameter("color.height").as_int();
  config.color_fps = get_parameter("color.fps").as_int();
  config.depth_width = get_parameter("depth.width").as_int();
  config.depth_height = get_parameter("depth.height").as_int();
  config.depth_fps = get_parameter("depth.fps").as_int();
  config.ir_width = get_parameter("ir.width").as_int();
  config.ir_height = get_parameter("ir.height").as_int();
  config.ir_fps = get_parameter("ir.fps").as_int();
  config.sync_mode = get_parameter("sync_mode").as_string();
  config.frame_queue_size = get_parameter("frame_queue_size").as_int();

  switch (output_mode_) {
    case OutputMode::RGBD:
    case OutputMode::RGBDepth:
      config.color_enabled = true; config.depth_enabled = true; break;
    case OutputMode::ColorOnly: config.color_enabled = true; break;
    case OutputMode::DepthOnly: config.depth_enabled = true; break;
    case OutputMode::IR:        config.ir_enabled = true; break;
    case OutputMode::All:
      config.color_enabled = true; config.depth_enabled = true; config.ir_enabled = true; break;
  }

  backend_->startStreaming(
    config,
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
}

void VxlCameraNode::publishMetadata(
  const rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr & pub,
  const BackendFramePtr & frame,
  const std::string & frame_id)
{
  if (!pub || !frame || !frame->isValid()) {
    return;
  }
  vxl_camera_msgs::msg::Metadata msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id;
  msg.timestamp_us = frame->timestamp_us;
  msg.frame_number = frame->sequence;
  msg.exposure_us = frame->exposure_us;
  msg.gain = frame->gain;
  pub->publish(msg);
}

void VxlCameraNode::publishRGBD(
  const BackendFramePtr & color,
  const BackendFramePtr & depth)
{
  auto msg = vxl_camera_msgs::msg::RGBD();
  msg.header.stamp = now();
  msg.header.frame_id = tf_prefix_ + "vxl_camera_link";

  auto rgb_img = frameToImageMsg(color, tf_prefix_ + "vxl_color_optical_frame");
  auto depth_img = frameToImageMsg(depth, tf_prefix_ + "vxl_depth_optical_frame");

  if (rgb_img) {msg.rgb = *rgb_img;}
  if (depth_img) {msg.depth = *depth_img;}
  if (color_camera_info_) {
    msg.rgb_camera_info = *color_camera_info_;
    msg.rgb_camera_info.header.stamp = msg.header.stamp;
  }
  if (depth_camera_info_) {
    msg.depth_camera_info = *depth_camera_info_;
    msg.depth_camera_info.header.stamp = msg.header.stamp;
  }

  rgbd_pub_->publish(msg);
}

void VxlCameraNode::publishColor(const BackendFramePtr & frame)
{
  auto img = frameToImageMsg(frame, tf_prefix_ + "vxl_color_optical_frame");
  if (!img) {return;}

  auto info = color_camera_info_ ?
    std::make_shared<sensor_msgs::msg::CameraInfo>(*color_camera_info_) :
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->header = img->header;
  color_pub_.publish(*img, *info);
}

void VxlCameraNode::publishDepth(const BackendFramePtr & frame)
{
  auto img = frameToImageMsg(frame, tf_prefix_ + "vxl_depth_optical_frame");
  if (!img) {return;}

  auto info = depth_camera_info_ ?
    std::make_shared<sensor_msgs::msg::CameraInfo>(*depth_camera_info_) :
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->header = img->header;
  depth_pub_.publish(*img, *info);
}

void VxlCameraNode::publishIR(const BackendFramePtr & frame)
{
  auto img = frameToImageMsg(frame, tf_prefix_ + "vxl_ir_optical_frame");
  if (!img) {return;}

  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->header = img->header;
  ir_pub_.publish(*img, *info);
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
    color_img = frameToImageMsg(color, tf_prefix_ + "vxl_color_optical_frame");
  }

  // Submit to the worker — drop-old keeps latency low under sustained load.
  pc_generator_->submit(depth_img, color_img,
    tf_prefix_ + "vxl_depth_optical_frame");
}

sensor_msgs::msg::Image::SharedPtr VxlCameraNode::frameToImageMsg(
  const BackendFramePtr & frame,
  const std::string & frame_id)
{
  if (!frame || !frame->isValid()) {
    return nullptr;
  }

  auto encoding = vxlFormatToRosEncoding(frame->format);
  if (!encoding) {
    // The SDK backend already handles MJPEG/H264 decoding internally; if we
    // still see an unmappable format here, there's no further conversion path.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported frame format: %d (no further conversion attempted)",
      static_cast<int>(frame->format));
    return nullptr;
  }

  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header.stamp = now();
  msg->header.frame_id = frame_id;
  msg->width = frame->width;
  msg->height = frame->height;
  msg->step = frame->stride;
  msg->is_bigendian = false;
  msg->encoding = *encoding;
  msg->data = frame->data;  // copy
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

// Service callbacks
void VxlCameraNode::onGetDeviceInfo(
  const vxl_camera_msgs::srv::GetDeviceInfo::Request::SharedPtr /*req*/,
  vxl_camera_msgs::srv::GetDeviceInfo::Response::SharedPtr res)
{
  try {
    auto info = backend_->getDeviceInfo();
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

void VxlCameraNode::onGetOption(
  const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::GetInt32::Response::SharedPtr res)
{
  try {
    int option_id = std::stoi(req->option_name);
    auto option = static_cast<vxl_option_t>(option_id);
    // Option ID range partitions sensor: <100 = color, >=100 = depth.
    auto sensor_type = (option_id >= 100) ? vxl::SensorType::Depth : vxl::SensorType::Color;
    if (!backend_->isOptionSupported(sensor_type, option)) {
      res->success = false;
      res->message = "Option " + req->option_name + " not supported";
      return;
    }
    res->value = static_cast<int32_t>(backend_->getOption(sensor_type, option));
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void VxlCameraNode::onSetOption(
  const vxl_camera_msgs::srv::SetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::SetInt32::Response::SharedPtr res)
{
  try {
    int option_id = std::stoi(req->option_name);
    auto option = static_cast<vxl_option_t>(option_id);
    auto sensor_type = (option_id >= 100) ? vxl::SensorType::Depth : vxl::SensorType::Color;
    if (!backend_->isOptionSupported(sensor_type, option)) {
      res->success = false;
      res->message = "Option " + req->option_name + " not supported";
      return;
    }
    backend_->setOption(sensor_type, option, static_cast<float>(req->value));
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void VxlCameraNode::onHwReset(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  try {
    backend_->hwReset();
    res->success = true;
    res->message = "Hardware reset triggered";
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

rcl_interfaces::msg::SetParametersResult VxlCameraNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const auto & cold = coldParameters();
  const auto & table = dynamicOptionTable();

  for (const auto & param : params) {
    const auto & name = param.get_name();

    if (cold.count(name)) {
      result.successful = false;
      result.reason = "Parameter '" + name +
        "' cannot be changed at runtime; restart the node with the new value.";
      RCLCPP_WARN(get_logger(), "%s", result.reason.c_str());
      return result;
    }

    auto it = table.find(name);
    if (it != table.end()) {
      try {
        if (!backend_->isOptionSupported(it->second.sensor, it->second.option)) {
          result.successful = false;
          result.reason = "Option not supported by device: " + name;
          return result;
        }
        backend_->setOption(it->second.sensor, it->second.option,
          static_cast<float>(param.as_int()));
        RCLCPP_INFO(get_logger(), "Set %s = %ld", name.c_str(), param.as_int());
      } catch (const std::exception & e) {
        result.successful = false;
        result.reason = std::string("SDK error setting ") + name + ": " + e.what();
        RCLCPP_ERROR(get_logger(), "%s", result.reason.c_str());
        return result;
      }
      continue;
    }

    // Other parameters (e.g. point_cloud.color) are read on each frame, no action needed here.
    RCLCPP_DEBUG(get_logger(), "Parameter '%s' updated (no SDK binding)", name.c_str());
  }

  // Hot-reload point cloud filter when any related param changed.
  if (pc_generator_) {
    bool pc_changed = std::any_of(params.begin(), params.end(),
      [](const rclcpp::Parameter & p) {
        return p.get_name().rfind("point_cloud.", 0) == 0;
      });
    if (pc_changed) {
      PointCloudFilter f;
      f.min_z_m = static_cast<float>(get_parameter("point_cloud.min_z").as_double());
      f.max_z_m = static_cast<float>(get_parameter("point_cloud.max_z").as_double());
      f.decimation = get_parameter("point_cloud.decimation").as_int();
      f.organized = get_parameter("point_cloud.organized").as_bool();
      pc_generator_->setFilter(f);
    }
  }

  // Hot-reload depth filter chain when any filters.* param changed.
  // Use the override merge: get_parameter() inside this callback returns
  // stale values (commit happens after callback success).
  bool filters_changed = std::any_of(params.begin(), params.end(),
    [](const rclcpp::Parameter & p) {
      return p.get_name().rfind(kFilterParamPrefix, 0) == 0;
    });
  if (filters_changed) {
    auto fc = readFilterChainParams(*this);
    applyFilterParamOverrides(fc, params);
    backend_->setFilterChain(fc);
    RCLCPP_INFO(get_logger(), "Filter chain: %s", filterChainSummary(fc).c_str());
  }

  return result;
}

}  // namespace vxl_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vxl_camera::VxlCameraNode)
