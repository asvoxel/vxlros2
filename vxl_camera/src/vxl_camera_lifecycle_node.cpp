#include "vxl_camera/vxl_camera_lifecycle_node.hpp"
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

VxlCameraLifecycleNode::VxlCameraLifecycleNode(const rclcpp::NodeOptions & options)
: VxlCameraLifecycleNode(options, makeSdkCameraBackend())
{
}

VxlCameraLifecycleNode::VxlCameraLifecycleNode(
  const rclcpp::NodeOptions & options, CameraBackendPtr backend)
: rclcpp_lifecycle::LifecycleNode("vxl_camera", options),
  backend_(std::move(backend))
{
  if (!backend_) {throw std::runtime_error("CameraBackend must not be null");}
  declareParameters();
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
    backend_->setDeviceEventCallback(
      [this](const vxl::DeviceInfo & info, bool added) {onDeviceEvent(info, added);});
    device_present_.store(backend_->isOpen());
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
    if (color_pub_) {color_pub_->on_activate();}
    if (color_info_pub_) {color_info_pub_->on_activate();}
    if (depth_pub_) {depth_pub_->on_activate();}
    if (depth_info_pub_) {depth_info_pub_->on_activate();}
    if (ir_pub_) {ir_pub_->on_activate();}
    if (ir_info_pub_) {ir_info_pub_->on_activate();}
    if (aligned_depth_pub_) {aligned_depth_pub_->on_activate();}
    if (aligned_depth_info_pub_) {aligned_depth_info_pub_->on_activate();}
    if (color_meta_pub_) {color_meta_pub_->on_activate();}
    if (depth_meta_pub_) {depth_meta_pub_->on_activate();}
    if (ir_meta_pub_) {ir_meta_pub_->on_activate();}
    if (connection_state_pub_) {connection_state_pub_->on_activate();}

    // Latched extrinsics: republish now that the publisher is active.
    if (extrinsics_pub_) {
      if (auto ext = backend_->getExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color)) {
        vxl_camera_msgs::msg::Extrinsics msg;
        for (int i = 0; i < 9; i++) {msg.rotation[i] = static_cast<double>(ext->rotation[i]);}
        for (int i = 0; i < 3; i++) {
          msg.translation[i] = static_cast<double>(ext->translation[i]) / 1000.0;
        }
        extrinsics_pub_->publish(msg);
      } else {
        RCLCPP_WARN(get_logger(), "Extrinsics publish skipped: unavailable from backend");
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
  if (color_pub_) {color_pub_->on_deactivate();}
  if (color_info_pub_) {color_info_pub_->on_deactivate();}
  if (depth_pub_) {depth_pub_->on_deactivate();}
  if (depth_info_pub_) {depth_info_pub_->on_deactivate();}
  if (ir_pub_) {ir_pub_->on_deactivate();}
  if (ir_info_pub_) {ir_info_pub_->on_deactivate();}
  if (aligned_depth_pub_) {aligned_depth_pub_->on_deactivate();}
  if (aligned_depth_info_pub_) {aligned_depth_info_pub_->on_deactivate();}
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
  color_pub_.reset();
  color_info_pub_.reset();
  depth_pub_.reset();
  depth_info_pub_.reset();
  ir_pub_.reset();
  ir_info_pub_.reset();
  aligned_depth_pub_.reset();
  aligned_depth_info_pub_.reset();
  color_meta_pub_.reset();
  depth_meta_pub_.reset();
  ir_meta_pub_.reset();
  connection_state_pub_.reset();
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

  // Depth post-processing filters (host-side, see filter_chain.hpp).
  declare_parameter("filters.decimation.enabled", false);
  declare_parameter("filters.decimation.scale", 2);
  declare_parameter("filters.threshold.enabled", false);
  declare_parameter("filters.threshold.min_mm", 100);
  declare_parameter("filters.threshold.max_mm", 5000);
  declare_parameter("filters.spatial.enabled", false);
  declare_parameter("filters.spatial.magnitude", 2);
  declare_parameter("filters.spatial.alpha", 0.5);
  declare_parameter("filters.spatial.delta", 20.0);
  declare_parameter("filters.temporal.enabled", false);
  declare_parameter("filters.temporal.alpha", 0.4);
  declare_parameter("filters.temporal.delta", 20.0);
  declare_parameter("filters.hole_filling.enabled", false);
  declare_parameter("filters.hole_filling.mode", 0);

  // Device-side filters (VXL6X5 only — silently no-op on VXL435).
  declare_parameter("filters.device.denoise.enabled", false);
  declare_parameter("filters.device.denoise.level", 2);
  declare_parameter("filters.device.median.enabled", false);
  declare_parameter("filters.device.median.kernel_size", 3);
  declare_parameter("filters.device.outlier_removal.enabled", false);
  declare_parameter("sync_mode", "strict");
  declare_parameter("frame_queue_size", 4);
  declare_parameter("tf_prefix", "");
  declare_parameter("publish_tf", true);
  declare_parameter("auto_recover_on_reconnect", true);

  // Depth-to-color alignment (host-side reprojection via SDK vxl_dip).
  declare_parameter("align_depth.enabled", false);
  declare_parameter("align_depth.scale", 1.0);  // VXL435=1.0, VXL6X5=8.0, VXL605=16.0

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
  target_serial_ = get_parameter("device_serial").as_string();
  auto_recover_.store(get_parameter("auto_recover_on_reconnect").as_bool());
}

void VxlCameraLifecycleNode::declareDynamicOptions()
{
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

  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&VxlCameraLifecycleNode::onParameterChange, this, std::placeholders::_1));
}

void VxlCameraLifecycleNode::initDevice()
{
  if (!backend_->open(target_serial_)) {
    throw std::runtime_error(target_serial_.empty() ?
      "No VxlSense device found" : "Device not found: " + target_serial_);
  }

  auto info = backend_->getDeviceInfo();
  if (target_serial_.empty()) {
    target_serial_ = info.serial_number;  // bind for hotplug filtering
    RCLCPP_INFO(get_logger(), "Auto-selected device S/N: %s", target_serial_.c_str());
  }
  RCLCPP_INFO(get_logger(), "Device opened: %s (S/N: %s, FW: %s)",
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

  // Split Image + CameraInfo so each is a true LifecyclePublisher.
  // (image_transport::CameraPublisher takes rclcpp::Node*, not LifecycleNode.)
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

  // Aligned depth (color view). Always created when both color + depth are
  // configured; actual publishing is gated on align_depth.enabled.
  if (need_color && need_depth) {
    aligned_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "~/aligned_depth_to_color/image_raw", qos);
    aligned_depth_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "~/aligned_depth_to_color/camera_info", qos);
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

  // Push initial filter chain to backend (default: all off → no-op).
  backend_->setFilterChain(readFilterChainParams(*this));

  // Push initial align config.
  backend_->setAlignDepthToColor(
    get_parameter("align_depth.enabled").as_bool(),
    static_cast<float>(get_parameter("align_depth.scale").as_double()));
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
    auto info = backend_->getDeviceInfo();
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

  if (auto ext = backend_->getExtrinsics(vxl::SensorType::Color, vxl::SensorType::Depth)) {
    transforms[1].transform.translation.x = ext->translation[0] / 1000.0;
    transforms[1].transform.translation.y = ext->translation[1] / 1000.0;
    transforms[1].transform.translation.z = ext->translation[2] / 1000.0;
  }

  tf_static_broadcaster_->sendTransform(transforms);
}

void VxlCameraLifecycleNode::startStream()
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

  backend_->startStreaming(config,
    std::bind(&VxlCameraLifecycleNode::framesetCallback, this, std::placeholders::_1));
}

void VxlCameraLifecycleNode::stopStream()
{
  if (backend_) {backend_->stopStreaming();}
}

void VxlCameraLifecycleNode::shutdownDevice()
{
  if (backend_) {
    backend_->stopStreaming();
    backend_->close();
  }
}

// ─── Frame processing ────────────────────────────────────────────────────────

void VxlCameraLifecycleNode::framesetCallback(BackendFrameSetPtr frameset)
{
  if (!frameset || frameset->empty()) {return;}

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
      if (color_frame) {
        publishImage(color_pub_, color_info_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame",
          color_camera_info_);
        color_stats_.frames_published++;
      }
      if (depth_frame) {
        publishImage(depth_pub_, depth_info_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame",
          depth_camera_info_);
        depth_stats_.frames_published++;
      }
      break;
    case OutputMode::ColorOnly:
      if (color_frame) {
        publishImage(color_pub_, color_info_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame",
          color_camera_info_);
        color_stats_.frames_published++;
      }
      break;
    case OutputMode::DepthOnly:
      if (depth_frame) {
        publishImage(depth_pub_, depth_info_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame",
          depth_camera_info_);
        depth_stats_.frames_published++;
      }
      break;
    case OutputMode::IR:
      if (ir_frame) {
        publishImage(ir_pub_, ir_info_pub_, ir_frame, tf_prefix_ + "vxl_ir_optical_frame", nullptr);
        ir_stats_.frames_published++;
      }
      break;
    case OutputMode::All:
      if (color_frame) {
        publishImage(color_pub_, color_info_pub_, color_frame, tf_prefix_ + "vxl_color_optical_frame",
          color_camera_info_);
        color_stats_.frames_published++;
      }
      if (depth_frame) {
        publishImage(depth_pub_, depth_info_pub_, depth_frame, tf_prefix_ + "vxl_depth_optical_frame",
          depth_camera_info_);
        depth_stats_.frames_published++;
      }
      if (ir_frame) {
        publishImage(ir_pub_, ir_info_pub_, ir_frame, tf_prefix_ + "vxl_ir_optical_frame", nullptr);
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

  // Aligned depth: publish only when backend produced it (align_depth.enabled).
  if (frameset->aligned_depth && aligned_depth_pub_ &&
    aligned_depth_pub_->is_activated() && color_camera_info_)
  {
    auto img = frameToImageMsg(frameset->aligned_depth,
        tf_prefix_ + "vxl_color_optical_frame");
    if (img) {
      aligned_depth_pub_->publish(*img);
      if (aligned_depth_info_pub_ && aligned_depth_info_pub_->is_activated()) {
        sensor_msgs::msg::CameraInfo ci = *color_camera_info_;
        ci.header = img->header;
        aligned_depth_info_pub_->publish(ci);
      }
    }
  }
}

void VxlCameraLifecycleNode::publishRGBD(
  const BackendFramePtr & color, const BackendFramePtr & depth)
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
  const std::shared_ptr<ImagePub> & img_pub,
  const std::shared_ptr<CameraInfoPub> & info_pub,
  const BackendFramePtr & frame,
  const std::string & frame_id,
  const sensor_msgs::msg::CameraInfo::SharedPtr & info)
{
  if (!img_pub || !img_pub->is_activated()) {return;}
  auto img = frameToImageMsg(frame, frame_id);
  if (!img) {return;}
  img_pub->publish(*img);
  if (info_pub && info_pub->is_activated()) {
    sensor_msgs::msg::CameraInfo ci = info ? *info : sensor_msgs::msg::CameraInfo();
    ci.header = img->header;
    info_pub->publish(ci);
  }
}

void VxlCameraLifecycleNode::publishMetadata(
  const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Metadata>> & pub,
  const BackendFramePtr & frame,
  const std::string & frame_id)
{
  if (!pub || !frame || !frame->isValid()) {return;}
  vxl_camera_msgs::msg::Metadata msg;
  msg.header.stamp = now();
  msg.header.frame_id = frame_id;
  msg.timestamp_us = frame->timestamp_us;
  msg.frame_number = frame->sequence;
  msg.exposure_us = frame->exposure_us;
  msg.gain = frame->gain;
  pub->publish(msg);
}

sensor_msgs::msg::Image::SharedPtr VxlCameraLifecycleNode::frameToImageMsg(
  const BackendFramePtr & frame, const std::string & frame_id)
{
  if (!frame || !frame->isValid()) {return nullptr;}

  auto encoding = vxlFormatToRosEncoding(frame->format);
  if (!encoding) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Unsupported frame format: %d", static_cast<int>(frame->format));
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
  if (!present && state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_WARN(get_logger(), "Device disconnected; deactivating");
    deactivate();
  }
  // INACTIVE → device back → re-activate (only if we were previously active)
  else if (present && state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    auto_recover_.load() && was_active_before_disconnect_.load())
  {
    RCLCPP_INFO(get_logger(), "Device reconnected; re-activating");
    // Reopen the backend (SDK requires close/open after a disconnect).
    try {
      if (!backend_->isOpen()) {
        if (!backend_->open(target_serial_)) {
          RCLCPP_ERROR(get_logger(), "Reopen failed for serial %s", target_serial_.c_str());
          return;  // try again next tick
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Reopen failed: %s", e.what());
      return;
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
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED: state_label = "UNCONFIGURED"; break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:     state_label = "INACTIVE"; break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:       state_label = "ACTIVE"; break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:    state_label = "FINALIZED"; break;
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

void VxlCameraLifecycleNode::onGetOption(
  const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr req,
  vxl_camera_msgs::srv::GetInt32::Response::SharedPtr res)
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
    res->value = static_cast<int32_t>(backend_->getOption(sensor_type, option));
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

void VxlCameraLifecycleNode::onHwReset(
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
        if (!backend_->isOptionSupported(it->second.sensor, it->second.option)) {
          result.successful = false;
          result.reason = "Option not supported: " + name;
          return result;
        }
        backend_->setOption(it->second.sensor, it->second.option,
          static_cast<float>(param.as_int()));
        RCLCPP_INFO(get_logger(), "Set %s = %ld", name.c_str(), param.as_int());
      } catch (const std::exception & e) {
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

  // Hot-reload point cloud filter (same stale-read merge as filter_chain).
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
      for (const auto & p : params) {
        const auto & n = p.get_name();
        if (n == "point_cloud.min_z") {f.min_z_m = static_cast<float>(p.as_double());}
        else if (n == "point_cloud.max_z") {f.max_z_m = static_cast<float>(p.as_double());}
        else if (n == "point_cloud.decimation") {f.decimation = static_cast<int>(p.as_int());}
        else if (n == "point_cloud.organized") {f.organized = p.as_bool();}
      }
      pc_generator_->setFilter(f);
    }
  }

  // Hot-reload depth-to-color alignment.
  bool align_changed = std::any_of(params.begin(), params.end(),
    [](const rclcpp::Parameter & p) {
      return p.get_name().rfind("align_depth.", 0) == 0;
    });
  if (align_changed) {
    bool enabled = get_parameter("align_depth.enabled").as_bool();
    float scale = static_cast<float>(get_parameter("align_depth.scale").as_double());
    for (const auto & p : params) {
      const auto & n = p.get_name();
      if (n == "align_depth.enabled") {enabled = p.as_bool();}
      else if (n == "align_depth.scale") {scale = static_cast<float>(p.as_double());}
    }
    backend_->setAlignDepthToColor(enabled, scale);
    RCLCPP_INFO(get_logger(), "align_depth: enabled=%d scale=%.2f",
      static_cast<int>(enabled), scale);
  }

  // Hot-reload depth filter chain. See vxl_camera_node.cpp for the why on
  // applyFilterParamOverrides — get_parameter() inside on_set is stale.
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
  if (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
      std::string("Lifecycle state: ") +
      (state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ? "INACTIVE" : "OTHER"));
  }

  if (color_meta_pub_) {report("color", color_stats_, get_parameter("color.fps").as_int());}
  if (depth_meta_pub_) {report("depth", depth_stats_, get_parameter("depth.fps").as_int());}
  if (ir_meta_pub_)    {report("ir",    ir_stats_,    get_parameter("ir.fps").as_int());}

  stat.add("device.connected", device_present_.load());
  stat.add("output_mode", get_parameter("output_mode").as_string());
  try {
    auto info = backend_->getDeviceInfo();
    stat.add("device.name", info.name);
    stat.add("device.serial", info.serial_number);
  } catch (const std::exception &) {}
}

}  // namespace vxl_camera

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vxl_camera::VxlCameraLifecycleNode)
