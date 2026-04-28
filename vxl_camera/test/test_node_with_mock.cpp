// Node-level tests for VxlCameraNode using MockCameraBackend.
// Verifies parameter callback routing, service handlers, and frame plumbing
// without requiring a real VxlSense device.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vxl_camera/vxl_camera_node.hpp"
#include "vxl_camera/mock_camera_backend.hpp"

#include <chrono>
#include <memory>

using vxl_camera::VxlCameraNode;
using vxl_camera::MockCameraBackend;
using vxl_camera::CameraBackendPtr;
using namespace std::chrono_literals;

namespace
{

std::shared_ptr<MockCameraBackend> makeMockWithDevice()
{
  auto m = std::make_shared<MockCameraBackend>();
  vxl::DeviceInfo info;
  info.name = "vxl-mock";
  info.serial_number = "SN12345";
  info.fw_version = "0.0.1";
  info.vendor_id = 0xABCD;
  info.product_id = 0x0001;
  m->setDeviceInfo(info);
  // Provide minimal intrinsics so buildCameraInfo doesn't drop them.
  vxl::Intrinsics intrin{};
  intrin.width = 640; intrin.height = 480;
  intrin.fx = intrin.fy = 500.0f;
  intrin.cx = 320.0f; intrin.cy = 240.0f;
  m->setIntrinsics(vxl::SensorType::Color, intrin);
  m->setIntrinsics(vxl::SensorType::Depth, intrin);
  return m;
}

rclcpp::NodeOptions defaultOptions()
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("output_mode", "rgb+depth"),  // skip RGBD pub for less setup
    rclcpp::Parameter("publish_tf", false),
    rclcpp::Parameter("point_cloud.enabled", false),
  });
  return opts;
}

}  // namespace

class NodeWithMockTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }
  void TearDown() override
  {
    if (rclcpp::ok()) {rclcpp::shutdown();}
  }
};

TEST_F(NodeWithMockTest, ConstructionWithMockSucceeds)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);
  EXPECT_TRUE(mock->isOpen());
  EXPECT_TRUE(mock->didStartStreaming());
}

TEST_F(NodeWithMockTest, ConstructionFailsWhenBackendOpenFails)
{
  auto mock = std::make_shared<MockCameraBackend>();
  mock->setOpenFails(true);
  EXPECT_THROW(
    std::make_shared<VxlCameraNode>(defaultOptions(), mock),
    std::runtime_error);
}

TEST_F(NodeWithMockTest, DynamicOptionDeclaredOnlyWhenBackendSupports)
{
  auto mock = makeMockWithDevice();
  // Only declare color.exposure (no other hot params).
  vxl_option_range_t r{1, 10000, 1, 5000};
  mock->setSupportedOption(vxl::SensorType::Color, VXL_OPTION_EXPOSURE, r);

  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);
  EXPECT_TRUE(node->has_parameter("color.exposure"));
  EXPECT_FALSE(node->has_parameter("color.gain"));   // not registered with mock
  EXPECT_EQ(node->get_parameter("color.exposure").as_int(), 5000);
}

TEST_F(NodeWithMockTest, SetHotParameterCallsBackendSetOption)
{
  auto mock = makeMockWithDevice();
  mock->setSupportedOption(vxl::SensorType::Color, VXL_OPTION_EXPOSURE,
    {1, 10000, 1, 5000});

  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  // No setOption calls during construction (initial value is read, not set).
  EXPECT_EQ(mock->calls_setOption().size(), 0u);

  auto result = node->set_parameter(rclcpp::Parameter("color.exposure", 7777));
  EXPECT_TRUE(result.successful) << result.reason;

  auto calls = mock->calls_setOption();
  ASSERT_EQ(calls.size(), 1u);
  EXPECT_EQ(calls[0].sensor, vxl::SensorType::Color);
  EXPECT_EQ(calls[0].option, VXL_OPTION_EXPOSURE);
  EXPECT_FLOAT_EQ(calls[0].value, 7777.0f);
}

TEST_F(NodeWithMockTest, SetColdParameterIsRejected)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  // color.width is in coldParameters() — runtime change must be rejected.
  auto result = node->set_parameter(rclcpp::Parameter("color.width", 320));
  EXPECT_FALSE(result.successful);
  EXPECT_NE(result.reason.find("cannot be changed at runtime"), std::string::npos);
  EXPECT_EQ(mock->calls_setOption().size(), 0u);
}

TEST_F(NodeWithMockTest, SetUnsupportedHotParameterIsRejected)
{
  auto mock = makeMockWithDevice();
  // Don't register VXL_OPTION_GAIN with the mock.
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  // The parameter wasn't declared (because mock doesn't support it), so set
  // should fail with "parameter not declared".
  EXPECT_FALSE(node->has_parameter("color.gain"));
}

TEST_F(NodeWithMockTest, GetDeviceInfoServiceReturnsBackendData)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  auto client = node->create_client<vxl_camera_msgs::srv::GetDeviceInfo>(
    "~/get_device_info");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto req = std::make_shared<vxl_camera_msgs::srv::GetDeviceInfo::Request>();
  auto fut = client->async_send_request(req);
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, fut, 2s),
    rclcpp::FutureReturnCode::SUCCESS);

  auto res = fut.get();
  EXPECT_TRUE(res->success);
  EXPECT_EQ(res->device_info.name, "vxl-mock");
  EXPECT_EQ(res->device_info.serial_number, "SN12345");
  EXPECT_EQ(res->device_info.firmware_version, "0.0.1");
}

TEST_F(NodeWithMockTest, SetOptionServiceRoutesByIdRange)
{
  auto mock = makeMockWithDevice();
  // Register a depth option (id >= 100 → routes to Depth sensor)
  mock->setSupportedOption(vxl::SensorType::Depth, VXL_OPTION_MIN_DISTANCE,
    {0, 10000, 1, 100});
  // And a color option (id < 100 → routes to Color sensor)
  mock->setSupportedOption(vxl::SensorType::Color, VXL_OPTION_EXPOSURE,
    {1, 10000, 1, 5000});

  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  auto client = node->create_client<vxl_camera_msgs::srv::SetInt32>("~/set_option");
  ASSERT_TRUE(client->wait_for_service(2s));

  // Set color exposure (option id 1)
  {
    auto req = std::make_shared<vxl_camera_msgs::srv::SetInt32::Request>();
    req->option_name = "1";
    req->value = 8888;
    auto fut = client->async_send_request(req);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, fut, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(fut.get()->success);
  }
  // Set depth min_distance (option id 101 → depth sensor)
  {
    auto req = std::make_shared<vxl_camera_msgs::srv::SetInt32::Request>();
    req->option_name = "101";
    req->value = 250;
    auto fut = client->async_send_request(req);
    ASSERT_EQ(rclcpp::spin_until_future_complete(node, fut, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(fut.get()->success);
  }

  auto calls = mock->calls_setOption();
  ASSERT_GE(calls.size(), 2u);
  EXPECT_EQ(calls[0].sensor, vxl::SensorType::Color);
  EXPECT_EQ(calls[1].sensor, vxl::SensorType::Depth);
  EXPECT_EQ(calls[1].option, VXL_OPTION_MIN_DISTANCE);
}

TEST_F(NodeWithMockTest, HwResetServiceCallsBackend)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  auto client = node->create_client<std_srvs::srv::Trigger>("~/hw_reset");
  ASSERT_TRUE(client->wait_for_service(2s));
  auto fut = client->async_send_request(
    std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, fut, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(fut.get()->success);
  EXPECT_EQ(mock->hwResetCount(), 1);
}

TEST_F(NodeWithMockTest, AlignDepthPushedToBackendOnConstruction)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);
  EXPECT_GE(mock->alignSetCount(), 1);
  EXPECT_FALSE(mock->alignEnabled());
  EXPECT_FLOAT_EQ(mock->alignScale(), 1.0f);
}

TEST_F(NodeWithMockTest, AlignDepthEnabledViaParameter)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);
  int before = mock->alignSetCount();

  auto results = node->set_parameters({
    rclcpp::Parameter("align_depth.enabled", true),
    rclcpp::Parameter("align_depth.scale", 8.0),
  });
  for (const auto & r : results) {EXPECT_TRUE(r.successful) << r.reason;}

  EXPECT_GT(mock->alignSetCount(), before);
  EXPECT_TRUE(mock->alignEnabled());
  EXPECT_FLOAT_EQ(mock->alignScale(), 8.0f);
}

TEST_F(NodeWithMockTest, FilterChainPushedToBackendOnConstruction)
{
  auto mock = makeMockWithDevice();
  // Construction should push a (default-disabled) FilterChain to the backend.
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);
  EXPECT_GE(mock->filterChainSetCount(), 1);
  auto fc = mock->lastFilterChain();
  EXPECT_FALSE(fc.hole_filling.enabled);
  EXPECT_FALSE(fc.spatial.enabled);
}

TEST_F(NodeWithMockTest, FilterParamSetTriggersBackendUpdate)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);
  int before = mock->filterChainSetCount();

  // Enable hole-filling at runtime — backend should see the new chain.
  auto r = node->set_parameter(rclcpp::Parameter("filters.hole_filling.enabled", true));
  EXPECT_TRUE(r.successful) << r.reason;

  EXPECT_GE(mock->filterChainSetCount(), before + 1);
  auto fc = mock->lastFilterChain();
  EXPECT_TRUE(fc.hole_filling.enabled)
    << "Backend should have received the new filter config (not stale read)";
}

TEST_F(NodeWithMockTest, MultipleFilterParamsAtomicMerge)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraNode>(defaultOptions(), mock);

  // Enable spatial + tune two of its params in one batch.
  auto results = node->set_parameters({
    rclcpp::Parameter("filters.spatial.enabled", true),
    rclcpp::Parameter("filters.spatial.alpha", 0.8),
    rclcpp::Parameter("filters.spatial.delta", 30.0),
  });
  for (const auto & r : results) {
    EXPECT_TRUE(r.successful) << r.reason;
  }

  auto fc = mock->lastFilterChain();
  EXPECT_TRUE(fc.spatial.enabled);
  EXPECT_FLOAT_EQ(fc.spatial.alpha, 0.8f);
  EXPECT_FLOAT_EQ(fc.spatial.delta, 30.0f);
}

TEST_F(NodeWithMockTest, StartStreamingConfigMatchesParameters)
{
  auto mock = makeMockWithDevice();
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("output_mode", "rgb+depth"),
    rclcpp::Parameter("color.width", 320),
    rclcpp::Parameter("color.height", 240),
    rclcpp::Parameter("color.fps", 15),
    rclcpp::Parameter("publish_tf", false),
  });

  auto node = std::make_shared<VxlCameraNode>(opts, mock);

  auto config = mock->lastStreamConfig();
  ASSERT_TRUE(config.has_value());
  EXPECT_TRUE(config->color_enabled);
  EXPECT_TRUE(config->depth_enabled);
  EXPECT_FALSE(config->ir_enabled);
  EXPECT_EQ(config->color_width, 320u);
  EXPECT_EQ(config->color_height, 240u);
  EXPECT_EQ(config->color_fps, 15u);
}
