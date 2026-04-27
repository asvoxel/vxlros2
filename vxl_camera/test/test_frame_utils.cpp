#include <gtest/gtest.h>

#include "vxl_camera/frame_utils.hpp"

#include <cmath>

using vxl_camera::OutputMode;
using vxl_camera::parseOutputMode;
using vxl_camera::outputModeToString;
using vxl_camera::vxlFormatToRosEncoding;
using vxl_camera::buildCameraInfo;

// ─── parseOutputMode ─────────────────────────────────────────────────────────

TEST(FrameUtilsParseOutputMode, AllValidStringsParse)
{
  EXPECT_EQ(parseOutputMode("rgbd"),       OutputMode::RGBD);
  EXPECT_EQ(parseOutputMode("rgb+depth"),  OutputMode::RGBDepth);
  EXPECT_EQ(parseOutputMode("ir"),         OutputMode::IR);
  EXPECT_EQ(parseOutputMode("depth_only"), OutputMode::DepthOnly);
  EXPECT_EQ(parseOutputMode("color_only"), OutputMode::ColorOnly);
  EXPECT_EQ(parseOutputMode("all"),        OutputMode::All);
}

TEST(FrameUtilsParseOutputMode, UnknownReturnsNullopt)
{
  EXPECT_FALSE(parseOutputMode("").has_value());
  EXPECT_FALSE(parseOutputMode("RGBD").has_value());          // case-sensitive
  EXPECT_FALSE(parseOutputMode("depth").has_value());
  EXPECT_FALSE(parseOutputMode("rgbd ").has_value());          // trailing space
  EXPECT_FALSE(parseOutputMode("rgb_depth").has_value());      // wrong separator
}

TEST(FrameUtilsParseOutputMode, RoundTripMatches)
{
  for (auto m : {OutputMode::RGBD, OutputMode::RGBDepth, OutputMode::IR,
    OutputMode::DepthOnly, OutputMode::ColorOnly, OutputMode::All})
  {
    auto s = outputModeToString(m);
    EXPECT_EQ(parseOutputMode(s), m) << "failed for " << s;
  }
}

// ─── vxlFormatToRosEncoding ──────────────────────────────────────────────────

TEST(FrameUtilsEncoding, MappedFormatsReturnRosString)
{
  EXPECT_EQ(*vxlFormatToRosEncoding(vxl::Format::BGR),    "bgr8");
  EXPECT_EQ(*vxlFormatToRosEncoding(vxl::Format::RGB),    "rgb8");
  EXPECT_EQ(*vxlFormatToRosEncoding(vxl::Format::Z16),    "16UC1");
  EXPECT_EQ(*vxlFormatToRosEncoding(vxl::Format::Gray8),  "mono8");
  EXPECT_EQ(*vxlFormatToRosEncoding(vxl::Format::Gray16), "mono16");
}

TEST(FrameUtilsEncoding, UnmappedFormatsReturnNullopt)
{
  // Compressed / YUV formats cannot map directly; caller must convert via SDK.
  EXPECT_FALSE(vxlFormatToRosEncoding(vxl::Format::MJPEG).has_value());
  EXPECT_FALSE(vxlFormatToRosEncoding(vxl::Format::H264).has_value());
  EXPECT_FALSE(vxlFormatToRosEncoding(vxl::Format::YUYV).has_value());
  EXPECT_FALSE(vxlFormatToRosEncoding(vxl::Format::UYVY).has_value());
  EXPECT_FALSE(vxlFormatToRosEncoding(vxl::Format::NV12).has_value());
  EXPECT_FALSE(vxlFormatToRosEncoding(vxl::Format::Unknown).has_value());
}

// ─── buildCameraInfo ─────────────────────────────────────────────────────────

TEST(FrameUtilsCameraInfo, FillsKMatrixAndDimensions)
{
  vxl::Intrinsics intrin{};
  intrin.width = 1280;
  intrin.height = 720;
  intrin.fx = 600.5f;
  intrin.fy = 601.0f;
  intrin.cx = 640.0f;
  intrin.cy = 360.0f;
  for (int i = 0; i < 5; i++) {intrin.coeffs[i] = 0.1f * (i + 1);}

  auto info = buildCameraInfo(intrin, "color_optical_frame");

  EXPECT_EQ(info.header.frame_id, "color_optical_frame");
  EXPECT_EQ(info.width, 1280u);
  EXPECT_EQ(info.height, 720u);
  EXPECT_EQ(info.distortion_model, "plumb_bob");

  // K matrix layout: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
  EXPECT_FLOAT_EQ(info.k[0], 600.5);
  EXPECT_FLOAT_EQ(info.k[1], 0.0);
  EXPECT_FLOAT_EQ(info.k[2], 640.0);
  EXPECT_FLOAT_EQ(info.k[4], 601.0);
  EXPECT_FLOAT_EQ(info.k[5], 360.0);
  EXPECT_FLOAT_EQ(info.k[8], 1.0);

  // R is identity
  EXPECT_FLOAT_EQ(info.r[0], 1.0);
  EXPECT_FLOAT_EQ(info.r[4], 1.0);
  EXPECT_FLOAT_EQ(info.r[8], 1.0);
}

TEST(FrameUtilsCameraInfo, DistortionCoefficientsCopiedInOrder)
{
  vxl::Intrinsics intrin{};
  intrin.width = 100;
  intrin.height = 100;
  intrin.fx = intrin.fy = 50.0f;
  intrin.cx = intrin.cy = 50.0f;
  intrin.coeffs[0] = 0.1f;   // k1
  intrin.coeffs[1] = -0.2f;  // k2
  intrin.coeffs[2] = 0.001f; // p1
  intrin.coeffs[3] = -0.002f;// p2
  intrin.coeffs[4] = 0.05f;  // k3

  auto info = buildCameraInfo(intrin, "frame");

  ASSERT_EQ(info.d.size(), 5u);
  EXPECT_FLOAT_EQ(info.d[0], 0.1);
  EXPECT_FLOAT_EQ(info.d[1], -0.2);
  EXPECT_FLOAT_EQ(info.d[2], 0.001);
  EXPECT_FLOAT_EQ(info.d[3], -0.002);
  EXPECT_FLOAT_EQ(info.d[4], 0.05);
}

TEST(FrameUtilsCameraInfo, ProjectionMatrixMatchesK)
{
  vxl::Intrinsics intrin{};
  intrin.width = 640;
  intrin.height = 480;
  intrin.fx = 300.0f; intrin.fy = 300.0f;
  intrin.cx = 320.0f; intrin.cy = 240.0f;

  auto info = buildCameraInfo(intrin, "f");
  // P matrix layout: [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
  EXPECT_FLOAT_EQ(info.p[0], 300.0);
  EXPECT_FLOAT_EQ(info.p[2], 320.0);
  EXPECT_FLOAT_EQ(info.p[3], 0.0);  // Tx = 0 for monocular
  EXPECT_FLOAT_EQ(info.p[5], 300.0);
  EXPECT_FLOAT_EQ(info.p[6], 240.0);
  EXPECT_FLOAT_EQ(info.p[10], 1.0);
}
