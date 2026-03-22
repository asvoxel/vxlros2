#include "vxl_camera/point_cloud_generator.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

namespace vxl_camera
{

PointCloudGenerator::PointCloudGenerator()
: fx_(0), fy_(0), cx_(0), cy_(0), depth_scale_(1.0f)
{
}

void PointCloudGenerator::configure(
  const sensor_msgs::msg::CameraInfo & depth_info,
  float depth_scale_mm)
{
  fx_ = static_cast<float>(depth_info.k[0]);
  fy_ = static_cast<float>(depth_info.k[4]);
  cx_ = static_cast<float>(depth_info.k[2]);
  cy_ = static_cast<float>(depth_info.k[5]);
  depth_scale_ = depth_scale_mm;

  // Validate focal length — must be positive and non-trivial
  if (fx_ < 1.0f || fy_ < 1.0f) {
    configured_ = false;
    return;
  }

  configured_ = true;
}

sensor_msgs::msg::PointCloud2::UniquePtr PointCloudGenerator::generate(
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::Image * color,
  const std::string & frame_id) const
{
  if (!configured_) {
    return nullptr;
  }

  bool has_color = (color != nullptr) &&
    (color->width == depth.width) &&
    (color->height == depth.height);

  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp = depth.header.stamp;
  cloud->header.frame_id = frame_id;
  cloud->width = depth.width;
  cloud->height = depth.height;
  cloud->is_dense = false;
  cloud->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(*cloud);
  if (has_color) {
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");

  const uint16_t * depth_data = reinterpret_cast<const uint16_t *>(depth.data.data());
  const uint8_t * color_data = has_color ? color->data.data() : nullptr;

  // Determine color step (bytes per pixel)
  int color_bpp = 0;
  if (has_color) {
    if (color->encoding == "bgr8" || color->encoding == "rgb8") {
      color_bpp = 3;
    } else if (color->encoding == "bgra8" || color->encoding == "rgba8") {
      color_bpp = 4;
    }
    // If encoding is unsupported, color_bpp stays 0 and color is skipped
  }

  // Validate color buffer size to prevent out-of-bounds access
  if (has_color && color_bpp > 0) {
    size_t required = static_cast<size_t>(depth.height - 1) * color->step +
      static_cast<size_t>(depth.width) * color_bpp;
    if (color->data.size() < required) {
      has_color = false;  // Buffer too small, skip color
    }
  }

  float inv_fx = 1.0f / fx_;
  float inv_fy = 1.0f / fy_;

  for (uint32_t v = 0; v < depth.height; ++v) {
    for (uint32_t u = 0; u < depth.width; ++u, ++iter_x, ++iter_y, ++iter_z) {
      uint16_t d = depth_data[v * depth.width + u];

      if (d == 0) {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      } else {
        float z = static_cast<float>(d) * depth_scale_ / 1000.0f;  // to meters
        *iter_x = (static_cast<float>(u) - cx_) * z * inv_fx;
        *iter_y = (static_cast<float>(v) - cy_) * z * inv_fy;
        *iter_z = z;
      }
    }
  }

  // Fill color if available and format is supported
  if (has_color && color_bpp >= 3) {
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*cloud, "rgb");
    bool is_bgr = (color->encoding == "bgr8" || color->encoding == "bgra8");

    for (uint32_t v = 0; v < depth.height; ++v) {
      for (uint32_t u = 0; u < depth.width; ++u, ++iter_rgb) {
        size_t idx = (v * color->step) + u * color_bpp;
        if (is_bgr) {
          iter_rgb[0] = color_data[idx + 2];  // R
          iter_rgb[1] = color_data[idx + 1];  // G
          iter_rgb[2] = color_data[idx + 0];  // B
        } else {
          iter_rgb[0] = color_data[idx + 0];
          iter_rgb[1] = color_data[idx + 1];
          iter_rgb[2] = color_data[idx + 2];
        }
      }
    }
  }

  return cloud;
}

}  // namespace vxl_camera
