#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vxl_camera/point_cloud_generator.hpp"

#include <atomic>
#include <chrono>
#include <thread>

class PointCloudTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    generator_ = std::make_unique<vxl_camera::PointCloudGenerator>();
  }

  void TearDown() override
  {
    generator_.reset();
    rclcpp::shutdown();
  }

  sensor_msgs::msg::CameraInfo makeDepthInfo(
    uint32_t w, uint32_t h, double fx, double fy, double cx, double cy)
  {
    sensor_msgs::msg::CameraInfo info;
    info.width = w;
    info.height = h;
    info.k = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
    return info;
  }

  sensor_msgs::msg::Image makeDepthImage(uint32_t w, uint32_t h)
  {
    sensor_msgs::msg::Image img;
    img.width = w;
    img.height = h;
    img.encoding = "16UC1";
    img.step = w * 2;
    img.data.resize(w * h * 2, 0);
    return img;
  }

  std::unique_ptr<vxl_camera::PointCloudGenerator> generator_;
};

TEST_F(PointCloudTest, UnconfiguredReturnsNull)
{
  auto depth = makeDepthImage(640, 480);
  auto result = generator_->generate(depth, nullptr, "test_frame");
  EXPECT_EQ(result, nullptr);
}

TEST_F(PointCloudTest, GenerateEmptyDepth)
{
  auto info = makeDepthInfo(640, 480, 380.0, 380.0, 320.0, 240.0);
  generator_->configure(info, 1.0f);

  auto depth = makeDepthImage(640, 480);
  auto result = generator_->generate(depth, nullptr, "test_frame");

  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->width, 640u);
  EXPECT_EQ(result->height, 480u);
  EXPECT_EQ(result->header.frame_id, "test_frame");
  EXPECT_FALSE(result->is_dense);
}

TEST_F(PointCloudTest, GenerateWithValidDepth)
{
  auto info = makeDepthInfo(4, 4, 2.0, 2.0, 2.0, 2.0);
  generator_->configure(info, 1.0f);

  auto depth = makeDepthImage(4, 4);
  // Set center pixel to 1000mm
  uint16_t * data = reinterpret_cast<uint16_t *>(depth.data.data());
  data[2 * 4 + 2] = 1000;  // pixel (2,2) = 1000mm = 1.0m

  auto result = generator_->generate(depth, nullptr, "test_frame");
  ASSERT_NE(result, nullptr);

  // Result should be a 4x4 point cloud
  EXPECT_EQ(result->width, 4u);
  EXPECT_EQ(result->height, 4u);

  // The point at (2,2) with depth=1000mm should have z=1.0m
  // x = (u - cx) * z / fx = (2-2)*1.0/2.0 = 0
  // y = (v - cy) * z / fy = (2-2)*1.0/2.0 = 0
  // Access point data: row 2, col 2 = index 10
  size_t point_idx = 2 * 4 + 2;
  size_t byte_offset = point_idx * result->point_step;
  const float * point = reinterpret_cast<const float *>(
    &result->data[byte_offset]);

  EXPECT_FLOAT_EQ(point[0], 0.0f);   // x
  EXPECT_FLOAT_EQ(point[1], 0.0f);   // y
  EXPECT_FLOAT_EQ(point[2], 1.0f);   // z
}

TEST_F(PointCloudTest, ZeroDepthProducesNaN)
{
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);

  auto depth = makeDepthImage(2, 2);  // all zeros

  auto result = generator_->generate(depth, nullptr, "test_frame");
  ASSERT_NE(result, nullptr);

  // All points should be NaN since depth is 0
  const float * point = reinterpret_cast<const float *>(result->data.data());
  EXPECT_TRUE(std::isnan(point[0]));
  EXPECT_TRUE(std::isnan(point[1]));
  EXPECT_TRUE(std::isnan(point[2]));
}

TEST_F(PointCloudTest, GenerateWithColor)
{
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);

  auto depth = makeDepthImage(2, 2);
  uint16_t * ddata = reinterpret_cast<uint16_t *>(depth.data.data());
  ddata[0] = 500;

  sensor_msgs::msg::Image color;
  color.width = 2;
  color.height = 2;
  color.encoding = "rgb8";
  color.step = 6;
  color.data.resize(12, 0);
  color.data[0] = 255;  // R
  color.data[1] = 128;  // G
  color.data[2] = 64;   // B

  auto result = generator_->generate(depth, &color, "test_frame");
  ASSERT_NE(result, nullptr);

  // Should have xyz + rgb fields
  EXPECT_GE(result->fields.size(), 4u);
}

TEST_F(PointCloudTest, RangeFilterClipsFarPoints)
{
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);

  auto depth = makeDepthImage(2, 2);
  uint16_t * d = reinterpret_cast<uint16_t *>(depth.data.data());
  d[0] = 500;    // 0.5 m — kept
  d[1] = 5000;   // 5.0 m — clipped by max_z=2.0
  d[2] = 200;    // 0.2 m — clipped by min_z=0.3
  d[3] = 1000;   // 1.0 m — kept

  vxl_camera::PointCloudFilter f;
  f.min_z_m = 0.3f;
  f.max_z_m = 2.0f;
  f.organized = true;
  generator_->setFilter(f);

  auto result = generator_->generate(depth, nullptr, "test_frame");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->width, 2u);
  EXPECT_EQ(result->height, 2u);

  const float * pts = reinterpret_cast<const float *>(result->data.data());
  // Point 0 (0.5m, kept), Point 1 (5m, NaN), Point 2 (0.2m, NaN), Point 3 (1m, kept)
  EXPECT_FLOAT_EQ(pts[0 * 3 + 2], 0.5f);
  EXPECT_TRUE(std::isnan(pts[1 * 3 + 2]));
  EXPECT_TRUE(std::isnan(pts[2 * 3 + 2]));
  EXPECT_FLOAT_EQ(pts[3 * 3 + 2], 1.0f);
}

TEST_F(PointCloudTest, DecimationReducesGrid)
{
  auto info = makeDepthInfo(4, 4, 4.0, 4.0, 2.0, 2.0);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(4, 4);
  uint16_t * d = reinterpret_cast<uint16_t *>(depth.data.data());
  for (int i = 0; i < 16; i++) {d[i] = 1000;}

  vxl_camera::PointCloudFilter f;
  f.decimation = 2;
  f.organized = true;
  generator_->setFilter(f);

  auto result = generator_->generate(depth, nullptr, "test_frame");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->width, 2u);   // 4 / 2
  EXPECT_EQ(result->height, 2u);
}

TEST_F(PointCloudTest, DenseModeDropsInvalid)
{
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(2, 2);
  uint16_t * d = reinterpret_cast<uint16_t *>(depth.data.data());
  d[0] = 1000;
  // d[1..3] left at 0 → invalid

  vxl_camera::PointCloudFilter f;
  f.organized = false;  // dense mode
  generator_->setFilter(f);

  auto result = generator_->generate(depth, nullptr, "test_frame");
  ASSERT_NE(result, nullptr);
  EXPECT_TRUE(result->is_dense);
  EXPECT_EQ(result->height, 1u);
  EXPECT_EQ(result->width, 1u);  // only 1 valid point
}

TEST_F(PointCloudTest, AsyncSubmitInvokesCallback)
{
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);

  std::atomic<int> count{0};
  generator_->startAsync(
    [&count](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {
      if (cloud) {count++;}
    });
  EXPECT_TRUE(generator_->isRunning());

  auto depth = std::make_shared<sensor_msgs::msg::Image>(makeDepthImage(2, 2));
  uint16_t * d = reinterpret_cast<uint16_t *>(depth->data.data());
  d[0] = 1000;

  generator_->submit(depth, nullptr, "test");

  // Worker is async; give it a moment to consume.
  for (int i = 0; i < 50 && count.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  generator_->stopAsync();
  EXPECT_GT(count.load(), 0);
  EXPECT_FALSE(generator_->isRunning());
}
