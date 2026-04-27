#ifndef VXL_CAMERA__POINT_CLOUD_GENERATOR_HPP_
#define VXL_CAMERA__POINT_CLOUD_GENERATOR_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vxl_camera_msgs/msg/extrinsics.hpp>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace vxl_camera
{

// Runtime-tunable filters for point cloud generation.
struct PointCloudFilter {
  float min_z_m = 0.0f;     // skip points closer than this (meters); 0 = disabled
  float max_z_m = 0.0f;     // skip points farther than this (meters); 0 = disabled
  int decimation = 1;       // keep every Nth pixel in each axis; 1 = no decimation
  bool organized = true;    // true = preserve H×W with NaN; false = dense (NaN dropped)
};

class PointCloudGenerator
{
public:
  using PublishFn =
    std::function<void(sensor_msgs::msg::PointCloud2::UniquePtr)>;

  PointCloudGenerator();
  ~PointCloudGenerator();

  // Camera model + depth scale (mm → m factor).
  void configure(
    const sensor_msgs::msg::CameraInfo & depth_info,
    float depth_scale_mm);

  // Live filter update — safe to call any time, takes effect from the next frame.
  void setFilter(const PointCloudFilter & filter);
  PointCloudFilter getFilter() const;

  // Synchronous generation (kept for unit tests and simple callers).
  sensor_msgs::msg::PointCloud2::UniquePtr generate(
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::Image * color,
    const std::string & frame_id) const;

  // Async pipeline: spawn a worker thread that consumes a single-slot,
  // drop-old queue. Use submit() to enqueue; the worker calls publish_fn for
  // each generated cloud.
  void startAsync(PublishFn publish_fn);
  void stopAsync();
  bool isRunning() const;

  void submit(
    sensor_msgs::msg::Image::SharedPtr depth,
    sensor_msgs::msg::Image::SharedPtr color,
    const std::string & frame_id);

private:
  void workerLoop();

  // Camera model
  float fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  float depth_scale_ = 1.0f;
  bool configured_ = false;

  // Filter (mutable runtime config; protected by filter_mutex_)
  mutable std::mutex filter_mutex_;
  PointCloudFilter filter_;

  // Async
  PublishFn publish_fn_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  struct Pending {
    sensor_msgs::msg::Image::SharedPtr depth;
    sensor_msgs::msg::Image::SharedPtr color;
    std::string frame_id;
  };
  std::optional<Pending> pending_;
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__POINT_CLOUD_GENERATOR_HPP_
