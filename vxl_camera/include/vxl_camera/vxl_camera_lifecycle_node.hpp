#ifndef VXL_CAMERA__VXL_CAMERA_LIFECYCLE_NODE_HPP_
#define VXL_CAMERA__VXL_CAMERA_LIFECYCLE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <vxl_camera_msgs/msg/rgbd.hpp>
#include <vxl_camera_msgs/msg/extrinsics.hpp>
#include <vxl_camera_msgs/msg/metadata.hpp>
#include <vxl_camera_msgs/srv/get_device_info.hpp>
#include <vxl_camera_msgs/srv/get_int32.hpp>
#include <vxl_camera_msgs/srv/set_int32.hpp>

#include "vxl_camera/camera_backend.hpp"
#include "vxl_camera/point_cloud_generator.hpp"
#include "vxl_camera/frame_utils.hpp"

#include <vxl.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace vxl_camera
{

// Lifecycle-managed ASVXL camera driver with USB hotplug auto-recovery.
//
// State map (rclcpp_lifecycle):
//   UNCONFIGURED → INACTIVE → ACTIVE
//
// on_configure: open device, build publishers/services/diagnostics, declare dynamic options
// on_activate:  start streaming + per-frame publish
// on_deactivate: stop streaming
// on_cleanup:   close device, release resources
//
// Hotplug: an SDK device-event callback flips an atomic flag; a 1Hz monitor timer
// observes the flag and the current lifecycle state, then triggers transitions:
//   - device disappears while ACTIVE → deactivate (publishes resume on reconnect)
//   - device reappears while INACTIVE → activate
class VxlCameraLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using State = rclcpp_lifecycle::State;

  // Pub typealiases (public so the .cpp's out-of-class definitions can name them).
  using ImagePub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>;
  using CameraInfoPub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>;

  explicit VxlCameraLifecycleNode(const rclcpp::NodeOptions & options);
  // Test-friendly constructor: inject a mock or alternate backend.
  VxlCameraLifecycleNode(const rclcpp::NodeOptions & options, CameraBackendPtr backend);
  ~VxlCameraLifecycleNode() override;

  // Lifecycle transitions
  CallbackReturn on_configure(const State & previous) override;
  CallbackReturn on_activate(const State & previous) override;
  CallbackReturn on_deactivate(const State & previous) override;
  CallbackReturn on_cleanup(const State & previous) override;
  CallbackReturn on_shutdown(const State & previous) override;
  CallbackReturn on_error(const State & previous) override;

private:
  // Setup
  void declareParameters();
  void declareDynamicOptions();
  void initDevice();
  void buildPublishers();
  void buildServices();
  void buildDiagnostics();
  void publishStaticTFs();
  void startStream();
  void stopStream();
  void shutdownDevice();

  // Frame processing
  void framesetCallback(BackendFrameSetPtr frameset);
  void publishRGBD(const BackendFramePtr & color, const BackendFramePtr & depth);
  void publishImage(
    const std::shared_ptr<ImagePub> & img_pub,
    const std::shared_ptr<CameraInfoPub> & info_pub,
    const BackendFramePtr & frame,
    const std::string & frame_id,
    const sensor_msgs::msg::CameraInfo::SharedPtr & info);
  void publishMetadata(
    const std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Metadata>> & pub,
    const BackendFramePtr & frame,
    const std::string & frame_id);

  // Helpers
  // Non-const because RCLCPP_*_THROTTLE inside requires a mutable Clock
  // (LifecycleNode::get_clock() const returns ConstSharedPtr).
  sensor_msgs::msg::Image::SharedPtr frameToImageMsg(
    const BackendFramePtr & frame, const std::string & frame_id);

  // Hotplug
  void onDeviceEvent(const vxl::DeviceInfo & info, bool added);
  void monitorTick();
  void publishConnectionState();

  // Services
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

  // Dynamic parameters
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // Diagnostics
  void diagnosticsCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Backend abstraction (real SDK or mock for tests)
  CameraBackendPtr backend_;
  std::unique_ptr<PointCloudGenerator> pc_generator_;

  // Configuration
  OutputMode output_mode_ = OutputMode::RGBD;
  std::string tf_prefix_;
  bool publish_tf_ = true;
  std::string target_serial_;  // serial we're bound to (for hotplug filtering)

  // Lifecycle publishers — image_transport doesn't support LifecycleNode in
  // Humble, so we publish raw Image + CameraInfo separately. Loses transport
  // plugins (compressed/theora/etc.) but lifecycle activation works correctly.
  // ImagePub / CameraInfoPub typealiases declared in public: section above.
  std::shared_ptr<ImagePub> color_pub_;
  std::shared_ptr<CameraInfoPub> color_info_pub_;
  std::shared_ptr<ImagePub> depth_pub_;
  std::shared_ptr<CameraInfoPub> depth_info_pub_;
  std::shared_ptr<ImagePub> ir_pub_;
  std::shared_ptr<CameraInfoPub> ir_info_pub_;
  std::shared_ptr<ImagePub> aligned_depth_pub_;
  std::shared_ptr<CameraInfoPub> aligned_depth_info_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::RGBD>> rgbd_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>> pc_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Extrinsics>>
    extrinsics_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Metadata>>
    color_meta_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Metadata>>
    depth_meta_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<vxl_camera_msgs::msg::Metadata>>
    ir_meta_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>
    connection_state_pub_;

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

  // Parameter callback
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  // Per-stream stats (for diagnostics)
  struct StreamStats {
    std::atomic<uint64_t> frames_published{0};
    std::atomic<uint64_t> last_sample_count{0};
    std::chrono::steady_clock::time_point last_sample_time{};
  };
  StreamStats color_stats_;
  StreamStats depth_stats_;
  StreamStats ir_stats_;
  std::mutex stats_mutex_;

  // Hotplug
  std::atomic<bool> device_present_{false};
  std::atomic<bool> auto_recover_{true};       // re-activate after reconnect
  std::atomic<bool> was_active_before_disconnect_{false};
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__VXL_CAMERA_LIFECYCLE_NODE_HPP_
