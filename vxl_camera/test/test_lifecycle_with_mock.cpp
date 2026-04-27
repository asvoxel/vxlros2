// Lifecycle-node tests with MockCameraBackend.
// Verifies state transitions, hotplug auto-recovery, and service handlers.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vxl_camera/vxl_camera_lifecycle_node.hpp"
#include "vxl_camera/mock_camera_backend.hpp"

#include <chrono>
#include <memory>

using vxl_camera::VxlCameraLifecycleNode;
using vxl_camera::MockCameraBackend;
using namespace std::chrono_literals;

namespace
{

std::shared_ptr<MockCameraBackend> makeMockWithDevice()
{
  auto m = std::make_shared<MockCameraBackend>();
  vxl::DeviceInfo info;
  info.name = "vxl-mock";
  info.serial_number = "SN-LC-1";
  info.fw_version = "0.0.1";
  m->setDeviceInfo(info);
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
    rclcpp::Parameter("output_mode", "rgb+depth"),
    rclcpp::Parameter("publish_tf", false),
  });
  return opts;
}

}  // namespace

class LifecycleWithMockTest : public ::testing::Test
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

TEST_F(LifecycleWithMockTest, StartsInUnconfigured)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraLifecycleNode>(defaultOptions(), mock);

  EXPECT_EQ(node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(mock->isOpen());
}

TEST_F(LifecycleWithMockTest, ConfigureOpensBackendAndDeclaresOptions)
{
  auto mock = makeMockWithDevice();
  mock->setSupportedOption(vxl::SensorType::Color, VXL_OPTION_EXPOSURE,
    {1, 10000, 1, 5000});

  auto node = std::make_shared<VxlCameraLifecycleNode>(defaultOptions(), mock);
  auto state = node->configure();
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(mock->isOpen());
  EXPECT_TRUE(node->has_parameter("color.exposure"));
}

TEST_F(LifecycleWithMockTest, ActivateStartsStreamingDeactivateStops)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraLifecycleNode>(defaultOptions(), mock);

  ASSERT_EQ(node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(mock->isStreaming());
  EXPECT_EQ(mock->startStreamingCount(), 1);

  ASSERT_EQ(node->deactivate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_FALSE(mock->isStreaming());
  EXPECT_EQ(mock->stopStreamingCount(), 1);
}

TEST_F(LifecycleWithMockTest, CleanupClosesBackendAndResetsPubs)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraLifecycleNode>(defaultOptions(), mock);

  ASSERT_EQ(node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->cleanup().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_FALSE(mock->isOpen());
}

TEST_F(LifecycleWithMockTest, HotplugDisconnectTriggersDeactivate)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraLifecycleNode>(defaultOptions(), mock);

  ASSERT_EQ(node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Simulate USB unplug
  vxl::DeviceInfo info; info.serial_number = "SN-LC-1";
  mock->emitDeviceEvent(info, false);

  // Drive the executor to fire the 1Hz monitor timer.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  auto end = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < end &&
    node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_EQ(node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(LifecycleWithMockTest, HotplugReconnectAutoActivates)
{
  auto mock = makeMockWithDevice();
  auto node = std::make_shared<VxlCameraLifecycleNode>(defaultOptions(), mock);

  ASSERT_EQ(node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  vxl::DeviceInfo info; info.serial_number = "SN-LC-1";

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());

  // Disconnect → wait for INACTIVE
  mock->emitDeviceEvent(info, false);
  auto t1 = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < t1 &&
    node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_EQ(node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // Reconnect → wait for ACTIVE
  mock->emitDeviceEvent(info, true);
  auto t2 = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < t2 &&
    node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_GE(mock->startStreamingCount(), 2);  // initial + after reconnect
}

TEST_F(LifecycleWithMockTest, HotplugIgnoresOtherSerials)
{
  // Ensure events for a different serial don't affect our node.
  auto mock = makeMockWithDevice();
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("device_serial", "SN-LC-1"),
    rclcpp::Parameter("publish_tf", false),
  });
  auto node = std::make_shared<VxlCameraLifecycleNode>(opts, mock);

  ASSERT_EQ(node->configure().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Some OTHER device unplugged — should be ignored.
  vxl::DeviceInfo other; other.serial_number = "DIFFERENT-SN";
  mock->emitDeviceEvent(other, false);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  auto end = std::chrono::steady_clock::now() + 1500ms;
  while (std::chrono::steady_clock::now() < end) {
    exec.spin_some();
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_EQ(node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}
