#include "vxl_camera/point_cloud_generator.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace vxl_camera
{

PointCloudGenerator::PointCloudGenerator() = default;

PointCloudGenerator::~PointCloudGenerator()
{
  stopAsync();
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

  // Validate focal length — must be positive and non-trivial.
  configured_ = (fx_ >= 1.0f && fy_ >= 1.0f);
}

void PointCloudGenerator::setFilter(const PointCloudFilter & filter)
{
  std::lock_guard<std::mutex> lock(filter_mutex_);
  filter_ = filter;
  if (filter_.decimation < 1) {filter_.decimation = 1;}
}

PointCloudFilter PointCloudGenerator::getFilter() const
{
  std::lock_guard<std::mutex> lock(filter_mutex_);
  return filter_;
}

sensor_msgs::msg::PointCloud2::UniquePtr PointCloudGenerator::generate(
  const sensor_msgs::msg::Image & depth,
  const sensor_msgs::msg::Image * color,
  const std::string & frame_id) const
{
  if (!configured_) {return nullptr;}

  // Snapshot the live filter so a concurrent setFilter() doesn't tear the values mid-frame.
  PointCloudFilter f;
  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    f = filter_;
  }

  bool has_color = (color != nullptr) &&
    (color->width == depth.width) && (color->height == depth.height);

  int color_bpp = 0;
  bool is_bgr = false;
  if (has_color) {
    if (color->encoding == "bgr8") {color_bpp = 3; is_bgr = true;}
    else if (color->encoding == "rgb8") {color_bpp = 3;}
    else if (color->encoding == "bgra8") {color_bpp = 4; is_bgr = true;}
    else if (color->encoding == "rgba8") {color_bpp = 4;}

    // Defensive size check
    if (color_bpp > 0) {
      size_t required = static_cast<size_t>(depth.height - 1) * color->step +
        static_cast<size_t>(depth.width) * color_bpp;
      if (color->data.size() < required) {has_color = false;}
    } else {
      has_color = false;
    }
  }

  uint32_t out_w = (depth.width + f.decimation - 1) / f.decimation;
  uint32_t out_h = (depth.height + f.decimation - 1) / f.decimation;

  auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  cloud->header.stamp = depth.header.stamp;
  cloud->header.frame_id = frame_id;
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

  const uint16_t * depth_data = reinterpret_cast<const uint16_t *>(depth.data.data());
  const uint8_t * color_data = has_color ? color->data.data() : nullptr;

  const float inv_fx = 1.0f / fx_;
  const float inv_fy = 1.0f / fy_;
  const float min_z = f.min_z_m;
  const float max_z = f.max_z_m;

  if (f.organized) {
    // Organized: width × height matches the (decimated) depth grid; invalid points are NaN.
    cloud->width = out_w;
    cloud->height = out_h;
    cloud->is_dense = false;
    modifier.resize(static_cast<size_t>(out_w) * out_h);

    sensor_msgs::PointCloud2Iterator<float> jx(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> jy(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> jz(*cloud, "z");
    std::optional<sensor_msgs::PointCloud2Iterator<uint8_t>> jrgb;
    if (has_color) {jrgb.emplace(*cloud, "rgb");}

    for (uint32_t v = 0; v < depth.height; v += f.decimation) {
      for (uint32_t u = 0; u < depth.width; u += f.decimation) {
        uint16_t d = depth_data[v * depth.width + u];
        float z = static_cast<float>(d) * depth_scale_ / 1000.0f;
        bool keep = (d != 0) &&
          (min_z <= 0.0f || z >= min_z) &&
          (max_z <= 0.0f || z <= max_z);
        if (keep) {
          *jx = (static_cast<float>(u) - cx_) * z * inv_fx;
          *jy = (static_cast<float>(v) - cy_) * z * inv_fy;
          *jz = z;
        } else {
          *jx = *jy = *jz = std::numeric_limits<float>::quiet_NaN();
        }
        if (jrgb) {
          if (keep) {
            size_t idx = (v * color->step) + u * color_bpp;
            (*jrgb)[0] = color_data[idx + (is_bgr ? 2 : 0)];  // R
            (*jrgb)[1] = color_data[idx + 1];                  // G
            (*jrgb)[2] = color_data[idx + (is_bgr ? 0 : 2)];  // B
          } else {
            (*jrgb)[0] = (*jrgb)[1] = (*jrgb)[2] = 0;
          }
          ++(*jrgb);
        }
        ++jx; ++jy; ++jz;
      }
    }
  } else {
    // Dense: only valid points are emitted; size is unknown until we walk the grid.
    cloud->is_dense = true;
    modifier.resize(static_cast<size_t>(out_w) * out_h);  // upper bound
    sensor_msgs::PointCloud2Iterator<float> jx(*cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> jy(*cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> jz(*cloud, "z");
    std::optional<sensor_msgs::PointCloud2Iterator<uint8_t>> jrgb;
    if (has_color) {jrgb.emplace(*cloud, "rgb");}

    size_t kept = 0;
    for (uint32_t v = 0; v < depth.height; v += f.decimation) {
      for (uint32_t u = 0; u < depth.width; u += f.decimation) {
        uint16_t d = depth_data[v * depth.width + u];
        if (d == 0) {continue;}
        float z = static_cast<float>(d) * depth_scale_ / 1000.0f;
        if ((min_z > 0.0f && z < min_z) || (max_z > 0.0f && z > max_z)) {continue;}
        *jx = (static_cast<float>(u) - cx_) * z * inv_fx;
        *jy = (static_cast<float>(v) - cy_) * z * inv_fy;
        *jz = z;
        if (jrgb) {
          size_t idx = (v * color->step) + u * color_bpp;
          (*jrgb)[0] = color_data[idx + (is_bgr ? 2 : 0)];
          (*jrgb)[1] = color_data[idx + 1];
          (*jrgb)[2] = color_data[idx + (is_bgr ? 0 : 2)];
          ++(*jrgb);
        }
        ++jx; ++jy; ++jz;
        ++kept;
      }
    }
    modifier.resize(kept);
    cloud->width = static_cast<uint32_t>(kept);
    cloud->height = 1;
  }

  return cloud;
}

// ─── Async pipeline ──────────────────────────────────────────────────────────

void PointCloudGenerator::startAsync(PublishFn publish_fn)
{
  if (running_.load()) {return;}
  publish_fn_ = std::move(publish_fn);
  running_.store(true);
  worker_ = std::thread([this]() {workerLoop();});
}

void PointCloudGenerator::stopAsync()
{
  if (!running_.load()) {return;}
  running_.store(false);
  queue_cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_.reset();
  }
  publish_fn_ = nullptr;
}

bool PointCloudGenerator::isRunning() const
{
  return running_.load();
}

void PointCloudGenerator::submit(
  sensor_msgs::msg::Image::SharedPtr depth,
  sensor_msgs::msg::Image::SharedPtr color,
  const std::string & frame_id)
{
  if (!running_.load()) {return;}
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    // Drop-old: overwrite any unprocessed pending frame.
    pending_ = Pending{std::move(depth), std::move(color), frame_id};
  }
  queue_cv_.notify_one();
}

void PointCloudGenerator::workerLoop()
{
  while (running_.load()) {
    Pending job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {return pending_.has_value() || !running_.load();});
      if (!running_.load()) {return;}
      job = std::move(*pending_);
      pending_.reset();
    }
    if (!job.depth) {continue;}
    auto cloud = generate(*job.depth, job.color.get(), job.frame_id);
    if (cloud && publish_fn_) {publish_fn_(std::move(cloud));}
  }
}

}  // namespace vxl_camera
