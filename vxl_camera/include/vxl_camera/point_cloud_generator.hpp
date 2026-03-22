#ifndef VXL_CAMERA__POINT_CLOUD_GENERATOR_HPP_
#define VXL_CAMERA__POINT_CLOUD_GENERATOR_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vxl_camera_msgs/msg/extrinsics.hpp>
#include <string>

namespace vxl_camera
{

class PointCloudGenerator
{
public:
  PointCloudGenerator();

  void configure(
    const sensor_msgs::msg::CameraInfo & depth_info,
    float depth_scale_mm);

  sensor_msgs::msg::PointCloud2::UniquePtr generate(
    const sensor_msgs::msg::Image & depth,
    const sensor_msgs::msg::Image * color,
    const std::string & frame_id) const;

private:
  float fx_, fy_, cx_, cy_;
  float depth_scale_;  // mm to meters
  bool configured_ = false;
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__POINT_CLOUD_GENERATOR_HPP_
