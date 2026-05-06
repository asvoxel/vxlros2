#ifndef VXL_CAMERA__VXL_CAMERA_NODE_HPP_
#define VXL_CAMERA__VXL_CAMERA_NODE_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>

#include <vxl_camera_msgs/msg/rgbd.hpp>
#include <vxl_camera_msgs/msg/extrinsics.hpp>
#include <vxl_camera_msgs/msg/metadata.hpp>
#include <vxl_camera_msgs/srv/get_device_info.hpp>
#include <vxl_camera_msgs/srv/get_int32.hpp>
#include <vxl_camera_msgs/srv/set_int32.hpp>

#include "vxl_camera/camera_backend.hpp"
#include "vxl_camera/node_helpers.hpp"
#include "vxl_camera/point_cloud_generator.hpp"
#include "vxl_camera/frame_utils.hpp"

#include <vxl.hpp>

#include <atomic>
#include <memory>
#include <string>

namespace vxl_camera
{

class VxlCameraNode : public rclcpp::Node
{
public:
  // Default constructor uses the real SDK backend.
  explicit VxlCameraNode(const rclcpp::NodeOptions & options);
  // Test-friendly constructor: inject a mock or alternate backend.
  VxlCameraNode(const rclcpp::NodeOptions & options, CameraBackendPtr backend);
  ~VxlCameraNode() override;

private:
  // Initialization
  void declareParameters();
  void initDevice();
  void setupPublishers();
  void setupServices();
  void startStreaming();
  void shutdownDevice();

  // Frame processing
  void framesetCallback(BackendFrameSetPtr frameset);
  void publishRGBD(const BackendFramePtr & color, const BackendFramePtr & depth);
  void publishColor(const BackendFramePtr & frame);
  void publishDepth(const BackendFramePtr & frame);
  void publishIR(const BackendFramePtr & frame);
  void publishPointCloud(const BackendFramePtr & depth, const BackendFramePtr & color);
  void publishMetadata(
    const rclcpp::Publisher<vxl_camera_msgs::msg::Metadata>::SharedPtr & pub,
    const BackendFramePtr & frame,
    const std::string & frame_id);

  // Helpers
  // Returns UniquePtr so callers can std::move into publish for intra-process
  // zero-copy. Non-const because RCLCPP_*_THROTTLE requires a mutable Clock.
  sensor_msgs::msg::Image::UniquePtr frameToImageMsg(
    const BackendFramePtr & frame,
    const std::string & frame_id);
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

  // Backend abstraction (real SDK or mock for tests)
  CameraBackendPtr backend_;
  std::unique_ptr<PointCloudGenerator> pc_generator_;

  // Configuration
  OutputMode output_mode_;
  std::string tf_prefix_;
  bool publish_tf_;

  // Publishers — raw rclcpp::Publisher (not image_transport) so:
  //   1. Both VxlCameraNode and VxlCameraLifecycleNode advertise the same
  //      topic structure (image_transport doesn't support LifecycleNode in
  //      Humble, so the lifecycle node already had to use raw publishers;
  //      unifying gets rid of the per-variant downstream subscriber wart).
  //   2. Publishing via std::move(unique_ptr) enables intra-process zero-copy
  //      for composable subscribers in the same container — image_transport's
  //      publish(const Image&, ...) signature blocked that path.
  //   Tradeoff: lose image_transport's compressed/theora plugins. Users who
  //   need them can run a republisher node downstream.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ir_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ir_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr aligned_depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr aligned_depth_info_pub_;
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

  // Per-stream frame counters (used by ~/diagnostics 1Hz publisher).
  StreamDiagCounters color_diag_;
  StreamDiagCounters depth_diag_;
  StreamDiagCounters ir_diag_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__VXL_CAMERA_NODE_HPP_
