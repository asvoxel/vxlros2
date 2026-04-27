// Test-only executable: VxlCameraLifecycleNode wired to a MockCameraBackend
// + a background thread that emits synthetic framesets at ~30 fps. Used by
// test/test_lifecycle_launch.py to exercise the full ROS2 launch path
// without real hardware.
//
// NOT installed in production builds — gated behind BUILD_TESTING in CMakeLists.

#include <rclcpp/rclcpp.hpp>

#include "vxl_camera/vxl_camera_lifecycle_node.hpp"
#include "vxl_camera/mock_camera_backend.hpp"

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using vxl_camera::BackendFrame;
using vxl_camera::BackendFramePtr;
using vxl_camera::BackendFrameSet;
using vxl_camera::BackendFrameSetPtr;
using vxl_camera::MockCameraBackend;

namespace
{

BackendFramePtr makeFakeFrame(uint32_t w, uint32_t h, vxl::Format fmt, uint32_t seq)
{
  auto f = std::make_shared<BackendFrame>();
  f->width = w;
  f->height = h;
  f->format = fmt;
  // Bytes per pixel: Z16/Gray16 = 2, BGR/RGB = 3, others ignored here.
  size_t bpp =
    (fmt == vxl::Format::Z16 || fmt == vxl::Format::Gray16) ? 2 :
    (fmt == vxl::Format::BGR || fmt == vxl::Format::RGB) ? 3 : 1;
  f->stride = static_cast<uint32_t>(w * bpp);
  f->data.assign(f->stride * h, 0);
  f->sequence = seq;
  f->timestamp_us = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
  f->exposure_us = 5000;
  f->gain = 100;
  return f;
}

class FakeFrameSource
{
public:
  explicit FakeFrameSource(std::shared_ptr<MockCameraBackend> mock)
  : mock_(std::move(mock))
  {
    thread_ = std::thread([this]() {pump();});
  }

  ~FakeFrameSource()
  {
    running_ = false;
    if (thread_.joinable()) {thread_.join();}
  }

private:
  void pump()
  {
    uint32_t seq = 0;
    while (running_) {
      if (mock_->isStreaming()) {
        auto fs = std::make_shared<BackendFrameSet>();
        fs->color = makeFakeFrame(640, 480, vxl::Format::BGR, seq);
        fs->depth = makeFakeFrame(640, 480, vxl::Format::Z16, seq);
        mock_->emitFrameSet(fs);
        ++seq;
      }
      std::this_thread::sleep_for(33ms);  // ~30 fps
    }
  }

  std::shared_ptr<MockCameraBackend> mock_;
  std::atomic<bool> running_{true};
  std::thread thread_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  bool auto_activate = true;
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--no-auto-activate") {auto_activate = false;}
  }

  auto mock = std::make_shared<MockCameraBackend>();

  // Pre-configure a plausible device.
  vxl::DeviceInfo info;
  info.name = "vxl-mock";
  info.serial_number = "MOCK-SN-001";
  info.fw_version = "0.0.1-mock";
  info.vendor_id = 0xABCD;
  info.product_id = 0x0001;
  mock->setDeviceInfo(info);

  vxl::Intrinsics intrin{};
  intrin.width = 640;
  intrin.height = 480;
  intrin.fx = intrin.fy = 500.0f;
  intrin.cx = 320.0f;
  intrin.cy = 240.0f;
  mock->setIntrinsics(vxl::SensorType::Color, intrin);
  mock->setIntrinsics(vxl::SensorType::Depth, intrin);

  vxl::Extrinsics ext{};
  ext.rotation[0] = ext.rotation[4] = ext.rotation[8] = 1.0f;  // identity
  mock->setExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color, ext);
  mock->setExtrinsics(vxl::SensorType::Color, vxl::SensorType::Depth, ext);

  // Register one hot option so dynamic-parameter declaration has something to do.
  mock->setSupportedOption(vxl::SensorType::Color, VXL_OPTION_EXPOSURE,
    {1.0f, 10000.0f, 1.0f, 5000.0f});

  auto node = std::make_shared<vxl_camera::VxlCameraLifecycleNode>(
    rclcpp::NodeOptions(), mock);

  if (auto_activate) {
    auto cfg = node->configure();
    if (cfg.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "auto-configure failed");
      rclcpp::shutdown();
      return 1;
    }
    auto act = node->activate();
    if (act.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_FATAL(node->get_logger(), "auto-activate failed");
      rclcpp::shutdown();
      return 2;
    }
  }

  FakeFrameSource frames(mock);  // pump synthetic framesets

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
