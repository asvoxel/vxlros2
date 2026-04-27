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

TEST_F(PointCloudTest, BgrEncodingSwapsRedAndBlue)
{
  // Verify that BGR input produces correctly-ordered RGB output in the cloud.
  auto info = makeDepthInfo(1, 1, 100.0, 100.0, 0.5, 0.5);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(1, 1);
  reinterpret_cast<uint16_t *>(depth.data.data())[0] = 1000;

  sensor_msgs::msg::Image color;
  color.width = 1;
  color.height = 1;
  color.encoding = "bgr8";
  color.step = 3;
  color.data = {0x10, 0x20, 0x30};  // B=0x10, G=0x20, R=0x30 in BGR layout

  auto result = generator_->generate(depth, &color, "f");
  ASSERT_NE(result, nullptr);

  // Locate the rgb field offset and verify the bytes were swapped to RGB order.
  const sensor_msgs::msg::PointField * rgb_field = nullptr;
  for (const auto & f : result->fields) {
    if (f.name == "rgb") {rgb_field = &f;}
  }
  ASSERT_NE(rgb_field, nullptr);
  const uint8_t * rgb = result->data.data() + rgb_field->offset;
  EXPECT_EQ(rgb[0], 0x30);  // R from BGR[2]
  EXPECT_EQ(rgb[1], 0x20);  // G from BGR[1]
  EXPECT_EQ(rgb[2], 0x10);  // B from BGR[0]
}

TEST_F(PointCloudTest, RgbEncodingPreservesOrder)
{
  auto info = makeDepthInfo(1, 1, 100.0, 100.0, 0.5, 0.5);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(1, 1);
  reinterpret_cast<uint16_t *>(depth.data.data())[0] = 1000;

  sensor_msgs::msg::Image color;
  color.width = 1; color.height = 1; color.encoding = "rgb8";
  color.step = 3;
  color.data = {0x10, 0x20, 0x30};  // R=0x10, G=0x20, B=0x30

  auto result = generator_->generate(depth, &color, "f");
  ASSERT_NE(result, nullptr);
  const sensor_msgs::msg::PointField * rgb_field = nullptr;
  for (const auto & f : result->fields) {
    if (f.name == "rgb") {rgb_field = &f;}
  }
  ASSERT_NE(rgb_field, nullptr);
  const uint8_t * rgb = result->data.data() + rgb_field->offset;
  EXPECT_EQ(rgb[0], 0x10);
  EXPECT_EQ(rgb[1], 0x20);
  EXPECT_EQ(rgb[2], 0x30);
}

TEST_F(PointCloudTest, Bgra8FourChannelHandled)
{
  auto info = makeDepthInfo(1, 1, 100.0, 100.0, 0.5, 0.5);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(1, 1);
  reinterpret_cast<uint16_t *>(depth.data.data())[0] = 1000;

  sensor_msgs::msg::Image color;
  color.width = 1; color.height = 1; color.encoding = "bgra8";
  color.step = 4;
  color.data = {0x11, 0x22, 0x33, 0xFF};  // BGRA

  auto result = generator_->generate(depth, &color, "f");
  ASSERT_NE(result, nullptr);
  // Just check the cloud has the rgb field — bgra parsed without size mismatch.
  bool has_rgb = false;
  for (const auto & f : result->fields) {if (f.name == "rgb") {has_rgb = true;}}
  EXPECT_TRUE(has_rgb);
}

TEST_F(PointCloudTest, UnknownColorEncodingFallsBackToXyzOnly)
{
  auto info = makeDepthInfo(1, 1, 100.0, 100.0, 0.5, 0.5);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(1, 1);
  reinterpret_cast<uint16_t *>(depth.data.data())[0] = 1000;

  sensor_msgs::msg::Image color;
  color.width = 1; color.height = 1; color.encoding = "yuv422";  // unsupported
  color.step = 4;
  color.data = {0, 0, 0, 0};

  auto result = generator_->generate(depth, &color, "f");
  ASSERT_NE(result, nullptr);
  // Should fall back to XYZ only — no rgb field present.
  for (const auto & f : result->fields) {
    EXPECT_NE(f.name, "rgb") << "rgb should not be added for unknown encoding";
  }
}

TEST_F(PointCloudTest, DecimationLargerThanImageProducesOnePoint)
{
  auto info = makeDepthInfo(4, 4, 4.0, 4.0, 2.0, 2.0);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(4, 4);
  reinterpret_cast<uint16_t *>(depth.data.data())[0] = 1000;

  vxl_camera::PointCloudFilter f;
  f.decimation = 8;  // larger than image dim
  f.organized = true;
  generator_->setFilter(f);

  auto result = generator_->generate(depth, nullptr, "f");
  ASSERT_NE(result, nullptr);
  // ceil(4/8) = 1, so a 1×1 organized cloud
  EXPECT_EQ(result->width, 1u);
  EXPECT_EQ(result->height, 1u);
}

TEST_F(PointCloudTest, AsyncDropOldKeepsOnlyLatest)
{
  // Stress: submit many frames before the worker can wake; verify the worker
  // ends up processing roughly the last submission, not all of them.
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);

  std::atomic<int> calls{0};
  std::atomic<int> last_seen_byte{0};
  generator_->startAsync(
    [&](sensor_msgs::msg::PointCloud2::UniquePtr cloud) {
      calls++;
      // Read the first byte of the X field to identify which submission was processed
      if (cloud && !cloud->data.empty()) {
        last_seen_byte.store(static_cast<int>(cloud->data[0]));
      }
      // Slow callback so the queue can build up
      std::this_thread::sleep_for(std::chrono::milliseconds(40));
    });

  // Submit 10 frames in a tight loop. Drop-old means the worker will see far fewer.
  for (int i = 0; i < 10; ++i) {
    auto depth = std::make_shared<sensor_msgs::msg::Image>(makeDepthImage(2, 2));
    reinterpret_cast<uint16_t *>(depth->data.data())[0] =
      static_cast<uint16_t>(100 * (i + 1));
    generator_->submit(depth, nullptr, "f");
  }

  // Wait for the worker to drain whatever it kept.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  generator_->stopAsync();

  EXPECT_GT(calls.load(), 0) << "Worker never ran";
  EXPECT_LT(calls.load(), 10) << "Drop-old should reduce the call count below the submit count";
}

TEST_F(PointCloudTest, MinGreaterThanMaxDropsAllPoints)
{
  // Pathological filter: min_z > max_z means no point passes the keep test.
  // Verify it doesn't crash and produces all-NaN organized output.
  auto info = makeDepthInfo(2, 2, 100.0, 100.0, 1.0, 1.0);
  generator_->configure(info, 1.0f);
  auto depth = makeDepthImage(2, 2);
  uint16_t * d = reinterpret_cast<uint16_t *>(depth.data.data());
  d[0] = d[1] = d[2] = d[3] = 1000;  // 1.0 m

  vxl_camera::PointCloudFilter f;
  f.min_z_m = 5.0f;
  f.max_z_m = 1.0f;  // contradictory
  f.organized = true;
  generator_->setFilter(f);

  auto result = generator_->generate(depth, nullptr, "f");
  ASSERT_NE(result, nullptr);
  const float * pts = reinterpret_cast<const float *>(result->data.data());
  for (int i = 0; i < 4; i++) {
    EXPECT_TRUE(std::isnan(pts[i * 3 + 2])) << "Point " << i << " z should be NaN";
  }
}
