#include "vxl_camera/vxl_camera_lifecycle_node.hpp"
#include "vxl_camera/sensor_options.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>

namespace vxl_camera
{

VxlCameraLifecycleNode::VxlCameraLifecycleNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("vxl_camera", options)
{
  declareParameters();
  // Param callback registered after dynamic options are declared (in on_configure).
  RCLCPP_INFO(get_logger(), "VxlCameraLifecycleNode created in UNCONFIGURED state");
}

VxlCameraLifecycleNode::~VxlCameraLifecycleNode()
{
  shutdownDevice();
}

// ─── Lifecycle transitions ───────────────────────────────────────────────────

VxlCameraLifecycleNode::CallbackReturn
VxlCameraLifecycleNode::on_configure(const State & /*previous*/)
{
  RCLCPP_INFO(get_logger(), "on_configure");
  try {
    initDevice();
    declareDynamicOptions();
    buildPublishers();
    buildServices();
    buildDiagnostics();
    if (publish_tf_) {
      publishStaticTFs();
    }

    // Hotplug: register SDK callback and a 1Hz monitor that triggers transitions.
    context_->setDeviceEventCallback(
      [this](const vxl::DeviceInfo & info, bool added) {onDeviceEvent(info, added);});
    device_present_.store(device_ && device_->isOpen());
    monitor_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {monitorTick();});

    publishConnectionState();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_configure failed: %s", e.what());
    shutdownDevice();
    return CallbackReturn::FAILURE;
  }
}

VxlCameraLifecycleNode::CallbackReturn
VxlCameraLifecycleNode::on_activate(const State & /*previous*/)
{
  RCLCPP_INFO(get_logger(), "on_activate");
  try {
    // Activate every LifecyclePublisher we created.
    if (rgbd_pub_) {rgbd_pub_->on_activate();}
    if (pc_pub_) {pc_pub_->on_activate();}
    if (extrinsics_pub_) {extrinsics_pub_->on_activate();}
    if (color_meta_pub_) {color_meta_pub_->on_activate();}
    if (depth_meta_pub_) {depth_meta_pub_->on_activate();}
    if (ir_meta_pub_) {ir_meta_pub_->on_activate();}
    if (connection_state_pub_) {connection_state_pub_->on_activate();}

    // Latched extrinsics: republish now that the publisher is active.
    if (extrinsics_pub_ && device_) {
      try {
        auto ext = device_->getExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color);
        vxl_camera_msgs::msg::Extrinsics msg;
        for (int i = 0; i < 9; i++) {msg.rotation[i] = static_cast<double>(ext.rotation[i]);}
        for (int i = 0; i < 3; i++) {
          msg.translation[i] = static_cast<double>(ext.translation[i]) / 1000.0;
        }
        extrinsics_pub_->publish(msg);
      } catch (const vxl::Error & e) {
        RCLCPP_WARN(get_logger(), "Extrinsics publish skipped: %s", e.what());
      }
    }

    // Start the point cloud worker thread (drop-old queue) before frames begin to arrive.
    if (pc_generator_ && pc_pub_) {
      pc_generator_->startAsync(
        [this](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {
          if (pc_pub_ && pc_pub_->is_activated()) {pc_pub_->publish(std::move(cloud));}
        });
    }

    startStream();
    publishConnectionState();
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "on_activate failed: %s", e.what());
    return CallbackReturn::FAILURE;
  }
}

VxlCameraLifecycleNode::CallbackReturn
VxlCameraLifecycleNode::on_deactivate(const State & /*previous*/)
{
  RCLCPP_INFO(get_logger(), "on_deactivate");
  // If we deactivated because the device went away, remember to re-activate on reconnect.
  // If the user manually deactivated (device still present), don't auto-recover later.
  was_active_before_disconnect_.store(!device_present_.load());
  stopStream();
  if (pc_generator_) {pc_generator_->stopAsync();}

  if (rgbd_pub_) {rgbd_pub_->on_deactivate();}
  if (pc_pub_) {pc_pub_->on_deactivate();}
  if (extrinsics_pub_) {extrinsics_pub_->on_deactivate();}
  if (color_meta_pub_) {color_meta_pub_->on_deactivate();}
  if (depth_meta_pub_) {depth_meta_pub_->on_deactivate();}
  if (ir_meta_pub_) {ir_meta_pub_->on_deactivate();}
  if (connection_state_pub_) {connection_state_pub_->on_deactivate();}

  publishConnectionState();
  return CallbackReturn::SUCCESS;
}

VxlCameraLifecycleNode::CallbackReturn
VxlCameraLifecycleNode::on_cleanup(const State & /*previous*/)
{
  RCLCPP_INFO(get_logger(), "on_cleanup");
  monitor_timer_.reset();
  diag_timer_.reset();
  diag_updater_.reset();
  shutdownDevice();

  // Drop publishers and services so they're recreated on the next on_configure.
  rgbd_pub_.reset();
  pc_pub_.reset();
  extrinsics_pub_.reset();
  color_meta_pub_.reset();
  depth_meta_pub_.reset();
  ir_meta_pub_.reset();
  connection_state_pub_.reset();
  color_pub_ = image_transport::CameraPublisher();
  depth_pub_ = image_transport::CameraPublisher();
  ir_pub_ = image_transport::CameraPublisher();
  device_info_srv_.reset();
  get_option_srv_.reset();
  set_option_srv_.reset();
  hw_reset_srv_.reset();
  tf_static_broadcaster_.reset();
  pc_generator_.reset();
  was_active_before_disconnect_.store(false);
  return CallbackReturn::SUCCESS;
}

VxlCameraLifecycleNode::CallbackReturn
VxlCameraLifecycleNode::on_shutdown(const State & /*previous*/)
{
  RCLCPP_INFO(get_logger(), "on_shutdown");
  stopStream();
  shutdownDevice();
  return CallbackReturn::SUCCESS;
}

VxlCameraLifecycleNode::CallbackReturn
VxlCameraLifecycleNode::on_error(const State & /*previous*/)
{
  RCLCPP_ERROR(get_logger(), "on_error: stopping stream and closing device");
  stopStream();
  shutdownDevice();
  return CallbackReturn::SUCCESS;
}

// ─── Setup ───────────────────────────────────────────────────────────────────

void VxlCameraLifecycleNode::declareParameters()
{
  declare_parameter("device_serial", "");
  declare_parameter("output_mode", "rgbd");
  declare_parameter("color.width", 1280);
  declare_parameter("color.height", 720);
  declare_parameter("color.fps", 30);
  declare_parameter("depth.width", 640);
  declare_parameter("depth.height", 480);
  declare_parameter("depth.fps", 30);
  declare_parameter("ir.width", 640);
  declare_parameter("ir.height", 480);
  declare_parameter("ir.fps", 30);
  declare_parameter("point_cloud.enabled", false);
  declare_parameter("point_cloud.color", true);
  declare_parameter("point_cloud.min_z", 0.0);
  declare_parameter("point_cloud.max_z", 0.0);
  declare_parameter("point_cloud.decimation", 1);
  declare_parameter("point_cloud.organized", true);
  declare_parameter("sync_mode", "strict");
  declare_parameter("frame_queue_size", 4);
  declare_parameter("tf_prefix", "");
  declare_parameter("publish_tf", true);
  declare_parameter("auto_recover_on_reconnect", true);

  output_mode_ = parseOutputMode(get_parameter("output_mode").as_string());
  tf_prefix_ = get_parameter("tf_prefix").as_string();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  target_serial_ = get_parameter("device_serial").as_string();
  auto_recover_.store(get_parameter("auto_recover_on_reconnect").as_bool());
}

void VxlCameraLifecycleNode::declareDynamicOptions()
{
  for (const auto & [name, binding] : dynamicOptionTable()) {
    try {
      auto sensor = device_->getSensor(binding.sensor);
      if (!sensor || !sensor->isOptionSupported(binding.option)) {continue;}

      auto range = sensor->getOptionRange(binding.option);
      float current = sensor->getOption(binding.option);

      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = std::string(vxl_option_string(binding.option));
      rcl_interfaces::msg::IntegerRange irange;
      irange.from_value = static_cast<int64_t>(range.min);
      irange.to_value = static_cast<int64_t>(range.max);
      irange.step = static_cast<uint64_t>(std::max(1.0f, range.step));
      desc.integer_range.push_back(irange);

      declare_parameter<int>(name, static_cast<int>(current), desc);
    } catch (const vxl::Error & e) {
      RCLCPP_DEBUG(get_logger(), "Skipping option '%s': %s", name.c_str(), e.what());
    }
  }

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&VxlCameraLifecycleNode::onParameterChange, this, std::placeholders::_1));
}

VxlCameraLifecycleNode::OutputMode
VxlCameraLifecycleNode::parseOutputMode(const std::string & s) const
{
  if (s == "rgbd") {return OutputMode::RGBD;}
  if (s == "rgb+depth") {return OutputMode::RGBDepth;}
  if (s == "ir") {return OutputMode::IR;}
  if (s == "depth_only") {return OutputMode::DepthOnly;}
  if (s == "color_only") {return OutputMode::ColorOnly;}
  if (s == "all") {return OutputMode::All;}
  RCLCPP_WARN(get_logger(), "Unknown output_mode '%s', defaulting to 'rgbd'", s.c_str());
  return OutputMode::RGBD;
}

void VxlCameraLifecycleNode::initDevice()
{
  context_ = vxl::Context::create();

  if (target_serial_.empty()) {
    if (context_->deviceCount() == 0) {
      throw std::runtime_error("No VxlSense device found");
    }
    device_ = context_->getDevice(0);
    auto info = device_->getInfo();
    target_serial_ = info.serial_number;  // bind to this one for hotplug
    RCLCPP_INFO(get_logger(), "Auto-selected device S/N: %s", target_serial_.c_str());
  } else {
    device_ = context_->findDeviceBySerial(target_serial_);
    if (!device_) {
      throw std::runtime_error("Device not found: " + target_serial_);
    }
  }

  device_->open();

  auto info = device_->getInfo();
  RCLCPP_INFO(get_logger(), "Device opened: %s (S/N: %s, FW: %s)",
    info.name.c_str(), info.serial_number.c_str(), info.fw_version.c_str());

  try {
    auto color_intrin = device_->getIntrinsics(vxl::SensorType::Color);
    color_camera_info_ = buildCameraInfo(color_intrin, tf_prefix_ + "vxl_color_optical_frame");
  } catch (const vxl::Error & e) {
    RCLCPP_WARN(get_logger(), "Color intrinsics unavailable: %s", e.what());
  }
  try {
    auto depth_intrin = device_->getIntrinsics(vxl::SensorType::Depth);
    depth_camera_info_ = buildCameraInfo(depth_intrin, tf_prefix_ + "vxl_depth_optical_frame");
  } catch (const vxl::Error & e) {
    RCLCPP_WARN(get_logger(), "Depth intrinsics unavailable: %s", e.what());
  }
}

void VxlCameraLifecycleNode::buildPublishers()
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
  bool need_ir = (output_mode_ == OutputMode::IR || output_mode_ == OutputMode::All);

  if (output_mode_ == OutputMode::RGBD) {
    rgbd_pub_ = create_publisher<vxl_camera_msgs::msg::RGBD>("~/rgbd", qos);
  }

  // image_transport::CameraPublisher is not lifecycle-aware. We only call
  // publish() while the stream is running (which only happens in ACTIVE state),
  // so messages don't leak in INACTIVE.
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

  if (need_color) {
    color_meta_pub_ = create_publisher<vxl_camera_msgs::msg::Metadata>("~/color/metadata", qos);
  }
  if (need_depth) {
    depth_meta_pub_ = create_publisher<vxl_camera_msgs::msg::Metadata>("~/depth/metadata", qos);
  }
  if (need_ir) {
    ir_meta_pub_ = create_publisher<vxl_camera_msgs::msg::Metadata>("~/ir/metadata", qos);
  }

  if (get_parameter("point_cloud.enabled").as_bool() && need_depth) {
    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/depth/points", qos);
    pc_generator_ = std::make_unique<PointCloudGenerator>();
    if (depth_camera_info_) {pc_generator_->configure(*depth_camera_info_, 1.0f);}

    PointCloudFilter filter;
    filter.min_z_m = static_cast<float>(get_parameter("point_cloud.min_z").as_double());
    filter.max_z_m = static_cast<float>(get_parameter("point_cloud.max_z").as_double());
    filter.decimation = get_parameter("point_cloud.decimation").as_int();
    filter.organized = get_parameter("point_cloud.organized").as_bool();
    pc_generator_->setFilter(filter);
  }

  extrinsics_pub_ = create_publisher<vxl_camera_msgs::msg::Extrinsics>(
    "~/extrinsics/depth_to_color", rclcpp::QoS(1).transient_local());

  connection_state_pub_ = create_publisher<std_msgs::msg::String>(
    "~/connection_state", rclcpp::QoS(1).transient_local());
}

void VxlCameraLifecycleNode::buildServices()
{
  device_info_srv_ = create_service<vxl_camera_msgs::srv::GetDeviceInfo>(
    "~/get_device_info",
    std::bind(&VxlCameraLifecycleNode::onGetDeviceInfo, this,
    std::placeholders::_1, std::placeholders::_2));
  get_option_srv_ = create_service<vxl_camera_msgs::srv::GetInt32>(
    "~/get_option",
    std::bind(&VxlCameraLifecycleNode::onGetOption, this,
    std::placeholders::_1, std::placeholders::_2));
  set_option_srv_ = create_service<vxl_camera_msgs::srv::SetInt32>(
    "~/set_option",
    std::bind(&VxlCameraLifecycleNode::onSetOption, this,
    std::placeholders::_1, std::placeholders::_2));
  hw_reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/hw_reset",
    std::bind(&VxlCameraLifecycleNode::onHwReset, this,
    std::placeholders::_1, std::placeholders::_2));
}

void VxlCameraLifecycleNode::buildDiagnostics()
{
  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(
    get_node_base_interface(),
    get_node_clock_interface(),
    get_node_logging_interface(),
    get_node_parameters_interface(),
    get_node_timers_interface(),
    get_node_topics_interface());

  std::string hw_id = "vxl_camera";
  try {
    auto info = device_->getInfo();
    hw_id = info.name + ":" + info.serial_number;
  } catch (const std::exception &) {}
  diag_updater_->setHardwareID(hw_id);
  diag_updater_->add("vxl_camera",
    std::bind(&VxlCameraLifecycleNode::diagnosticsCallback, this, std::placeholders::_1));

  auto now_tp = std::chrono::steady_clock::now();
  color_stats_.last_sample_time = now_tp;
  depth_stats_.last_sample_time = now_tp;
  ir_stats_.last_sample_time = now_tp;

  diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    diag_updater_->force_update();
  });
}

void VxlCameraLifecycleNode::publishStaticTFs()
{
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  std::string base_frame = tf_prefix_ + "vxl_camera_link";
  auto stamp = now();

  tf2::Quaternion optical_rotation;
  optical_rotation.setRPY(-M_PI / 2.0, 0.0, -M_PI / 2.0);

  auto make_tf = [&](const std::string & child) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = base_frame;
      tf.child_frame_id = child;
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

  try {
    auto ext = device_->getExtrinsics(vxl::SensorType::Color, vxl::SensorType::Depth);
    transforms[1].transform.translation.x = ext.translation[0] / 1000.0;
    transforms[1].transform.translation.y = ext.translation[1] / 1000.0;
    transforms[1].transform.translation.z = ext.translation[2] / 1000.0;
  } catch (const vxl::Error &) {}

  tf_static_broadcaster_->sendTransform(transforms);
}

void VxlCameraLifecycleNode::startStream()
{
  StreamParams params;
  params.color_width = get_parameter("color.width").as_int();
  params.color_height = get_parameter("color.height").as_int();
  params.color_fps = get_parameter("color.fps").as_int();
  params.depth_width = get_parameter("depth.width").as_int();
  params.depth_height = get_parameter("depth.height").as_int();
  params.depth_fps = get_parameter("depth.fps").as_int();
  params.ir_width = get_parameter("ir.width").as_int();
  params.ir_height = get_parameter("ir.height").as_int();
  params.ir_fps = get_parameter("ir.fps").as_int();
  params.sync_mode = get_parameter("sync_mode").as_string();
  params.frame_queue_size = get_parameter("frame_queue_size").as_int();

  stream_manager_ = std::make_unique<StreamManager>(device_, params);

  switch (output_mode_) {
    case OutputMode::RGBD:
    case OutputMode::RGBDepth:
      stream_manager_->enableColor();
      stream_manager_->enableDepth();
      break;
    case OutputMode::ColorOnly: stream_manager_->enableColor(); break;
    case OutputMode::DepthOnly: stream_manager_->enableDepth(); break;
    case OutputMode::IR:        stream_manager_->enableIR(); break;
    case OutputMode::All:
      stream_manager_->enableColor();
      stream_manager_->enableDepth();
      stream_manager_->enableIR();
      break;
  }

  stream_manager_->start(
    std::bind(&VxlCameraLifecycleNode::framesetCallback, this, std::placeholders::_1));
}

void VxlCameraLifecycleNode::stopStream()
{
  if (stream_manager_) {
    stream_manager_->stop();
    stream_manager_.reset();
  }
}

void VxlCameraLifecycleNode::shutdownDevice()
{
  stopStream();
  if (device_ && device_->isOpen()) {device_->close();}
  device_.reset();
  context_.reset();
}

// ─── Frame processing ────────────────────────────────────────────────────────

void VxlCameraLifecycleNode::framesetCallback(vxl::FrameSetPtr frameset)
{
  if (!frameset || frameset->empty()) {return;}

  auto color_frame = frameset->getColorFrame();
  auto depth_frame = frameset->getDepthFrame();
  auto ir_frame = frameset->getIRFrame();

  switch (output_mode_) {
    case OutputMode::RGBD:
      if (color_frame && depth_frame) {
        publishRGBD(color_frame, depth_frame);
        color_stats_.frames_published++;
        depth_stats_.frames_published++;
      }
      break;
    case OutputMode::RGBDepth:
      if (color_frame) {
        publishImage(color_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame",
          color_camera_info_);
        color_stats_.frames_published++;
      }
      if (depth_frame) {
        publishImage(depth_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame",
          depth_camera_info_);
        depth_stats_.frames_published++;
      }
      break;
    case OutputMode::ColorOnly:
      if (color_frame) {
        publishImage(color_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame",
          color_camera_info_);
        color_stats_.frames_published++;
      }
      break;
    case OutputMode::DepthOnly:
      if (depth_frame) {
        publishImage(depth_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame",
          depth_camera_info_);
        depth_stats_.frames_published++;
      }
      break;
    case OutputMode::IR:
      if (ir_frame) {
        publishImage(ir_pub_, ir_frame, tf_prefix_ + "vxl_ir_optical_frame", nullptr);
        ir_stats_.frames_published++;
      }
      break;
    case OutputMode::All:
      if (color_frame) {
        publishImage(color_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame",
          color_camera_info_);
        color_stats_.frames_published++;
      }
      if (depth_frame) {
        publishImage(depth_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame",
          depth_camera_info_);
        depth_stats_.frames_published++;
      }
      if (ir_frame) {
        publishImage(ir_pub_, ir_frame, tf_prefix_ + "vxl_ir_optical_frame", nullptr);
        ir_stats_.frames_published++;
      }
      break;
  }

  if (color_frame) {
    publishMetadata(color_meta_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame");
  }
  if (depth_frame) {
    publishMetadata(depth_meta_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame");
  }
  if (ir_frame) {
    publishMetadata(ir_meta_pub_, ir_frame, tf_prefix_ + "vxl_ir_optical_frame");
  }

  if (pc_pub_ && depth_frame && pc_generator_ && pc_generator_->isRunning()) {
    auto depth_img = frameToImageMsg(depth_frame, tf_prefix_ + "vxl_depth_optical_frame");
    if (depth_img) {
      bool use_color = get_parameter("point_cloud.color").as_bool();
      sensor_msgs::msg::Image::SharedPtr color_img;
      if (use_color && color_frame) {
        color_img = frameToImageMsg(color_frame, tf_prefix_ + "vxl_color_optical_frame");
      }
      pc_generator_->submit(depth_img, color_img,
        tf_prefix_ + "vxl_depth_optical_frame");
    }
  }
}

void VxlCameraLifecycleNode::publishRGBD(
  const vxl::FramePtr & color, const vxl::FramePtr & depth)
{
  if (!rgbd_pub_) {return;}
  vxl_camera_msgs::msg::RGBD msg;
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

void VxlCameraLifecycleNode::publishImage(
  const image_transport::CameraPublisher & pub,
  const vxl::FramePtr & frame,
  const std::string & frame_id,
  const sensor_msgs::msg::CameraInfo::SharedPtr & info)
{
  auto img = frameToImageMsg(frame, frame_id);
  if (!img) {return;}
  auto ci = info ?
    std::make_shared<sensor_msgs::msg::CameraInfo>(*info) :
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  ci->header = img->header;
  pub.publish(*img, *ci);
}

void VxlCameraLifecycleNode::publishMetadata(
  const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Metadata>> & pub,
  const vxl::FramePtr & frame,
  const std::string & frame_id)
{
  if (!pub || !frame || !frame->isValid()) {return;}
  auto m = frame->metadata();
  vxl_camera_msgs::msg::Metadata msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id;
  msg.timestamp_us = m.timestamp_us;
  msg.frame_number = m.sequence;
  msg.exposure_us = m.exposure_us;
  msg.gain = m.gain;
  pub->publish(msg);
}

sensor_msgs::msg::Image::SharedPtr VxlCameraLifecycleNode::frameToImageMsg(
  const vxl::FramePtr & frame, const std::string & frame_id) const
{
  if (!frame || !frame->isValid()) {return nullptr;}
  auto msg = std::make_shared<sensor_msgs::msg::Image>();
  msg->header.stamp = now();
  msg->header.frame_id = frame_id;
  msg->width = frame->width();
  msg->height = frame->height();
  msg->step = frame->stride();
  msg->is_bigendian = false;

  switch (frame->format()) {
    case vxl::Format::BGR:    msg->encoding = "bgr8"; break;
    case vxl::Format::RGB:    msg->encoding = "rgb8"; break;
    case vxl::Format::Z16:    msg->encoding = "16UC1"; break;
    case vxl::Format::Gray8:  msg->encoding = "mono8"; break;
    case vxl::Format::Gray16: msg->encoding = "mono16"; break;
    default:
      try {
        auto converted = frame->convert(vxl::Format::BGR);
        return frameToImageMsg(converted, frame_id);
      } catch (const vxl::Error & e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
          "Frame conversion failed: %s", e.what());
        return nullptr;
      }
  }

  size_t data_size = frame->dataSize();
  msg->data.resize(data_size);
  std::memcpy(msg->data.data(), frame->data(), data_size);
  return msg;
}

sensor_msgs::msg::CameraInfo::SharedPtr VxlCameraLifecycleNode::buildCameraInfo(
  const vxl::Intrinsics & intrin, const std::string & frame_id) const
{
  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->header.frame_id = frame_id;
  info->width = intrin.width;
  info->height = intrin.height;
  info->distortion_model = "plumb_bob";
  info->d.resize(5);
  for (int i = 0; i < 5; i++) {info->d[i] = intrin.coeffs[i];}
  info->k = {static_cast<double>(intrin.fx), 0.0, static_cast<double>(intrin.cx),
    0.0, static_cast<double>(intrin.fy), static_cast<double>(intrin.cy),
    0.0, 0.0, 1.0};
  info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info->p = {static_cast<double>(intrin.fx), 0.0, static_cast<double>(intrin.cx), 0.0,
    0.0, static_cast<double>(intrin.fy), static_cast<double>(intrin.cy), 0.0,
    0.0, 0.0, 1.0, 0.0};
  return info;
}

// ─── Hotplug ─────────────────────────────────────────────────────────────────

void VxlCameraLifecycleNode::onDeviceEvent(const vxl::DeviceInfo & info, bool added)
{
  // Called from SDK thread — keep the work minimal. Lifecycle transitions are
  // triggered from monitorTick() which runs in the executor thread.
  if (!target_serial_.empty() && info.serial_number != target_serial_) {
    return;
  }
  device_present_.store(added);
  RCLCPP_INFO(get_logger(), "Device %s %s", info.serial_number.c_str(),
    added ? "connected" : "DISCONNECTED");
}

void VxlCameraLifecycleNode::monitorTick()
{
  bool present = device_present_.load();
  auto state_id = get_current_state().id();

  // ACTIVE → device gone → deactivate
  if (!present && state_id == State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "Device disconnected; deactivating");
    deactivate();
  }
  // INACTIVE → device back → re-activate (only if we were previously active)
  else if (present && state_id == State::PRIMARY_STATE_INACTIVE &&
    auto_recover_.load() && was_active_before_disconnect_.load())
  {
    RCLCPP_INFO(get_logger(), "Device reconnected; re-activating");
    // Reopen device handle if needed (SDK requires close/open after a disconnect).
    try {
      if (device_ && !device_->isOpen()) {device_->open();}
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Reopen failed: %s", e.what());
      return;  // try again next tick
    }
    activate();
  }

  publishConnectionState();
}

void VxlCameraLifecycleNode::publishConnectionState()
{
  // Only valid in ACTIVE state (LifecyclePublisher drops with a log line otherwise).
  // The lifecycle state itself is always queryable via the standard
  // ~/get_state service for INACTIVE/UNCONFIGURED introspection.
  if (!connection_state_pub_ || !connection_state_pub_->is_activated()) {return;}
  std_msgs::msg::String msg;
  std::string state_label;
  switch (get_current_state().id()) {
    case State::PRIMARY_STATE_UNCONFIGURED: state_label = "UNCONFIGURED"; break;
    case State::PRIMARY_STATE_INACTIVE:     state_label = "INACTIVE"; break;
    case State::PRIMARY_STATE_ACTIVE:       state_label = "ACTIVE"; break;
    case State::PRIMARY_STATE_FINALIZED:    state_label = "FINALIZED"; break;
    default:                                state_label = "TRANSITIONING"; break;
  }
  msg.data = state_label + (device_present_.load() ? "/CONNECTED" : "/DISCONNECTED");
  connection_state_pub_->publish(msg);
}

// ─── Services ────────────────────────────────────────────────────────────────

void VxlCameraLifecycleNode::onGetDeviceInfo(
  const vxl_camera_msgs::srv::GetDeviceInfo::Request::SharedPtr /*req*/,
  vxl_camera_msgs::srv::GetDeviceInfo::Response::SharedPtr res)
{
  try {
    auto info = device_->getInfo();
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

void VxlCameraLifecycleNode::onGetOption(
  const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::GetInt32::Response::SharedPtr res)
{
  try {
    int option_id = std::stoi(req->option_name);
    auto option = static_cast<vxl_option_t>(option_id);
    auto sensor_type = (option_id >= 100) ? vxl::SensorType::Depth : vxl::SensorType::Color;
    auto sensor = device_->getSensor(sensor_type);
    if (!sensor->isOptionSupported(option)) {
      res->success = false;
      res->message = "Option " + req->option_name + " not supported";
      return;
    }
    res->value = static_cast<int32_t>(sensor->getOption(option));
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void VxlCameraLifecycleNode::onSetOption(
  const vxl_camera_msgs::srv::SetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::SetInt32::Response::SharedPtr res)
{
  try {
    int option_id = std::stoi(req->option_name);
    auto option = static_cast<vxl_option_t>(option_id);
    auto sensor_type = (option_id >= 100) ? vxl::SensorType::Depth : vxl::SensorType::Color;
    auto sensor = device_->getSensor(sensor_type);
    if (!sensor->isOptionSupported(option)) {
      res->success = false;
      res->message = "Option " + req->option_name + " not supported";
      return;
    }
    sensor->setOption(option, static_cast<float>(req->value));
    res->success = true;
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

void VxlCameraLifecycleNode::onHwReset(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  try {
    device_->hwReset();
    res->success = true;
    res->message = "Hardware reset triggered";
  } catch (const std::exception & e) {
    res->success = false;
    res->message = e.what();
  }
}

rcl_interfaces::msg::SetParametersResult
VxlCameraLifecycleNode::onParameterChange(const std::vector<rclcpp::Parameter> & params)
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
        "' cannot be changed at runtime; cleanup + reconfigure the node.";
      return result;
    }

    auto it = table.find(name);
    if (it != table.end()) {
      try {
        auto sensor = device_->getSensor(it->second.sensor);
        if (!sensor || !sensor->isOptionSupported(it->second.option)) {
          result.successful = false;
          result.reason = "Option not supported: " + name;
          return result;
        }
        sensor->setOption(it->second.option, static_cast<float>(param.as_int()));
        RCLCPP_INFO(get_logger(), "Set %s = %ld", name.c_str(), param.as_int());
      } catch (const vxl::Error & e) {
        result.successful = false;
        result.reason = std::string("SDK error: ") + e.what();
        return result;
      }
    }
  }

  // Refresh side-effects
  if (std::any_of(params.begin(), params.end(),
    [](const rclcpp::Parameter & p) {return p.get_name() == "auto_recover_on_reconnect";}))
  {
    auto_recover_.store(get_parameter("auto_recover_on_reconnect").as_bool());
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

  return result;
}

// ─── Diagnostics ─────────────────────────────────────────────────────────────

void VxlCameraLifecycleNode::diagnosticsCallback(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
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

    if (expected_fps > 0 && prev > 0 && fps > 0 && fps < expected_fps * 0.5) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
        std::string(name) + " fps below 50% of configured rate");
    }
  };

  auto state_id = get_current_state().id();
  if (state_id == State::PRIMARY_STATE_ACTIVE) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
      std::string("Lifecycle state: ") +
      (state_id == State::PRIMARY_STATE_INACTIVE ? "INACTIVE" : "OTHER"));
  }

  if (color_meta_pub_) {report("color", color_stats_, get_parameter("color.fps").as_int());}
  if (depth_meta_pub_) {report("depth", depth_stats_, get_parameter("depth.fps").as_int());}
  if (ir_meta_pub_)    {report("ir",    ir_stats_,    get_parameter("ir.fps").as_int());}

  stat.add("device.connected", device_present_.load());
  stat.add("output_mode", get_parameter("output_mode").as_string());
  if (device_) {
    try {
      auto info = device_->getInfo();
      stat.add("device.name", info.name);
      stat.add("device.serial", info.serial_number);
    } catch (const std::exception &) {}
  }
}

}  // namespace vxl_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vxl_camera::VxlCameraLifecycleNode)
