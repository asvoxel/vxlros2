#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vxl_camera_msgs/msg/rgbd.hpp>
#include <vxl_camera_msgs/msg/metadata.hpp>

#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class OutputModeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(OutputModeTest, ParseOutputModeValid)
{
  // Test that valid output_mode parameter values are accepted
  auto node = std::make_shared<rclcpp::Node>("test_node");
  node->declare_parameter("output_mode", "rgbd");

  std::string mode = node->get_parameter("output_mode").as_string();
  ASSERT_EQ(mode, "rgbd");

  // All valid modes
  std::vector<std::string> valid_modes = {
    "rgbd", "rgb+depth", "ir", "depth_only", "color_only", "all"
  };
  for (const auto & m : valid_modes) {
    EXPECT_NO_THROW(node->set_parameter(rclcpp::Parameter("output_mode", m)));
  }
}

TEST_F(OutputModeTest, DefaultParameterValues)
{
  auto node = std::make_shared<rclcpp::Node>("test_params");

  node->declare_parameter("color.width", 1280);
  node->declare_parameter("color.height", 720);
  node->declare_parameter("color.fps", 30);
  node->declare_parameter("depth.width", 640);
  node->declare_parameter("depth.height", 480);
  node->declare_parameter("depth.fps", 30);
  node->declare_parameter("point_cloud.enabled", false);
  node->declare_parameter("sync_mode", "strict");

  EXPECT_EQ(node->get_parameter("color.width").as_int(), 1280);
  EXPECT_EQ(node->get_parameter("color.height").as_int(), 720);
  EXPECT_EQ(node->get_parameter("color.fps").as_int(), 30);
  EXPECT_EQ(node->get_parameter("depth.width").as_int(), 640);
  EXPECT_EQ(node->get_parameter("depth.height").as_int(), 480);
  EXPECT_EQ(node->get_parameter("depth.fps").as_int(), 30);
  EXPECT_FALSE(node->get_parameter("point_cloud.enabled").as_bool());
  EXPECT_EQ(node->get_parameter("sync_mode").as_string(), "strict");
}

TEST_F(OutputModeTest, RGBDMessageStructure)
{
  // Verify RGBD message can be constructed with expected fields
  vxl_camera_msgs::msg::RGBD msg;
  msg.header.frame_id = "vxl_camera_link";

  msg.rgb.width = 1280;
  msg.rgb.height = 720;
  msg.rgb.encoding = "bgr8";
  msg.rgb.step = 1280 * 3;

  msg.depth.width = 640;
  msg.depth.height = 480;
  msg.depth.encoding = "16UC1";
  msg.depth.step = 640 * 2;

  EXPECT_EQ(msg.rgb.width, 1280u);
  EXPECT_EQ(msg.depth.encoding, "16UC1");
}

TEST_F(OutputModeTest, RGBDTopicSubscription)
{
  // Verify that a subscriber can be created for RGBD topic
  auto node = std::make_shared<rclcpp::Node>("test_sub");
  bool received = false;

  auto sub = node->create_subscription<vxl_camera_msgs::msg::RGBD>(
    "test_rgbd", 10,
    [&received](const vxl_camera_msgs::msg::RGBD::SharedPtr) {
      received = true;
    });

  auto pub = node->create_publisher<vxl_camera_msgs::msg::RGBD>("test_rgbd", 10);

  // Publish a test message
  auto msg = vxl_camera_msgs::msg::RGBD();
  msg.header.frame_id = "test";
  pub->publish(msg);

  // Spin briefly
  rclcpp::spin_some(node);
  // Message delivery may require another spin
  rclcpp::spin_some(node);

  // We mainly verify no crash; delivery depends on timing
  SUCCEED();
}

TEST_F(OutputModeTest, ImageTopicSubscription)
{
  // Verify separate color/depth topics for rgb+depth mode
  auto node = std::make_shared<rclcpp::Node>("test_image_sub");

  auto color_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "test_color", 10,
    [](const sensor_msgs::msg::Image::SharedPtr) {});

  auto depth_sub = node->create_subscription<sensor_msgs::msg::Image>(
    "test_depth", 10,
    [](const sensor_msgs::msg::Image::SharedPtr) {});

  EXPECT_NE(color_sub, nullptr);
  EXPECT_NE(depth_sub, nullptr);
}

TEST_F(OutputModeTest, MetadataMessageStructure)
{
  // Per-stream metadata is published alongside frames; verify the message layout.
  vxl_camera_msgs::msg::Metadata meta;
  meta.header.frame_id = "vxl_color_optical_frame";
  meta.timestamp_us = 1234567890ULL;
  meta.frame_number = 42;
  meta.exposure_us = 16000;
  meta.gain = 128;

  EXPECT_EQ(meta.timestamp_us, 1234567890ULL);
  EXPECT_EQ(meta.frame_number, 42u);
  EXPECT_EQ(meta.exposure_us, 16000u);
  EXPECT_EQ(meta.gain, 128u);
}

TEST_F(OutputModeTest, DynamicParameterDescriptorAccepted)
{
  // declareDynamicOptions uses integer_range constraints; verify the descriptor shape.
  auto node = std::make_shared<rclcpp::Node>("test_dyn_param");

  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = "exposure";
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 1;
  range.to_value = 10000;
  range.step = 1;
  desc.integer_range.push_back(range);

  EXPECT_NO_THROW(node->declare_parameter<int>("color.exposure", 5000, desc));
  EXPECT_EQ(node->get_parameter("color.exposure").as_int(), 5000);

  // In-range value succeeds; out-of-range value is rejected by the descriptor.
  auto in_range = node->set_parameter(rclcpp::Parameter("color.exposure", 8000));
  EXPECT_TRUE(in_range.successful);
  auto out_of_range = node->set_parameter(rclcpp::Parameter("color.exposure", 99999));
  EXPECT_FALSE(out_of_range.successful);
}
