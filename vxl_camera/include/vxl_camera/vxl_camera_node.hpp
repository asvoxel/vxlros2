#ifndef VXL_CAMERA__VXL_CAMERA_NODE_HPP_
#define VXL_CAMERA__VXL_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <vxl_camera_msgs/msg/rgbd.hpp>
#include <vxl_camera_msgs/msg/extrinsics.hpp>
#include <vxl_camera_msgs/msg/metadata.hpp>
#include <vxl_camera_msgs/srv/get_device_info.hpp>
#include <vxl_camera_msgs/srv/get_int32.hpp>
#include <vxl_camera_msgs/srv/set_int32.hpp>

#include "vxl_camera/stream_manager.hpp"
#include "vxl_camera/point_cloud_generator.hpp"

#include <vxl.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace vxl_camera
{

class VxlCameraNode : public rclcpp::Node
{
public:
  explicit VxlCameraNode(const rclcpp::NodeOptions & options);
  ~VxlCameraNode() override;

private:
  // Output modes
  enum class OutputMode {
    RGBD,        // Default: single synced RGBD topic
    RGBDepth,    // Separate color + depth topics
    IR,          // Debug: IR only
    DepthOnly,   // Debug: depth only
    ColorOnly,   // Debug: color only
    All          // Debug: all streams
  };

  // Initialization
  void declareParameters();
  void declareDynamicOptions();
  OutputMode parseOutputMode(const std::string & mode_str) const;
  void initDevice();
  void setupPublishers();
  void setupServices();
  void setupDiagnostics();
  void startStreaming();
  void shutdownDevice();

  // Frame processing
  void framesetCallback(vxl::FrameSetPtr frameset);
  void publishRGBD(const vxl::FramePtr & color, const vxl::FramePtr & depth);
  void publishColor(const vxl::FramePtr & frame);
  void publishDepth(const vxl::FramePtr & frame);
  void publishIR(const vxl::FramePtr & frame);
  void publishPointCloud(const vxl::FramePtr & depth, const vxl::FramePtr & color);
  void publishMetadata(
    const rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr & pub,
    const vxl::FramePtr & frame,
    const std::string & frame_id);

  // Diagnostics
  void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Helpers
  sensor_msgs::msg::Image::SharedPtr frameToImageMsg(
    const vxl::FramePtr & frame,
    const std::string & frame_id) const;
  sensor_msgs::msg::CameraInfo::SharedPtr buildCameraInfo(
    const vxl::Intrinsics & intrin,
    const std::string & frame_id) const;
  void publishStaticTFs();

  // Service callbacks
  void onGetDeviceInfo(
    const vxl_camera_msgs::srv::GetDeviceInfo::Request::SharedPtr req,
    vxl_camera_msgs::srv::GetDeviceInfo::Response::SharedPtr res);
  void onGetOption(
    const vxl_camera_msgs::srv::GetInt32::Request::SharedPtr req,
    vxl_camera_msgs::srv::GetInt32::Response::SharedPtr res);
  void onSetOption(
    const vxl_camera_msgs::srv::SetInt32::Request::SharedPtr req,
    vxl_camera_msgs::srv::SetInt32::Response::SharedPtr res);
  void onHwReset(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr res);

  // Dynamic parameter callback
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // VxlSense objects
  vxl::ContextPtr context_;
  vxl::DevicePtr device_;
  std::unique_ptr<StreamManager> stream_manager_;
  std::unique_ptr<PointCloudGenerator> pc_generator_;

  // Configuration
  OutputMode output_mode_;
  std::string tf_prefix_;
  bool publish_tf_;

  // Publishers
  image_transport::CameraPublisher color_pub_;
  image_transport::CameraPublisher depth_pub_;
  image_transport::CameraPublisher ir_pub_;
  rclcpp::Publisher<vxl_camera_msgs::msg::RGBD>::SharedPtr rgbd_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<vxl_camera_msgs::msg::Extrinsics>::SharedPtr extrinsics_pub_;
  rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr color_meta_pub_;
  rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr depth_meta_pub_;
  rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr ir_meta_pub_;

  // Camera info
  sensor_msgs::msg::CameraInfo::SharedPtr color_camera_info_;
  sensor_msgs::msg::CameraInfo::SharedPtr depth_camera_info_;

  // Services
  rclcpp::Service<vxl_camera_msgs::srv::GetDeviceInfo>::SharedPtr device_info_srv_;
  rclcpp::Service<vxl_camera_msgs::srv::GetInt32>::SharedPtr get_option_srv_;
  rclcpp::Service<vxl_camera_msgs::srv::SetInt32>::SharedPtr set_option_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hw_reset_srv_;

  // TF
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Per-stream frame counters (for diagnostics — published rate vs. expected)
  struct StreamStats {
    std::atomic<uint64_t> frames_published{0};
    std::atomic<uint64_t> frames_dropped{0};
    std::atomic<uint64_t> last_sample_count{0};
    std::chrono::steady_clock::time_point last_sample_time{};
  };
  StreamStats color_stats_;
  StreamStats depth_stats_;
  StreamStats ir_stats_;
  std::mutex stats_mutex_;
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__VXL_CAMERA_NODE_HPP_
