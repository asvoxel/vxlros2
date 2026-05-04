// Tests for SdkCameraBackend that don't require a real ASVXL camera.
// On CI without USB hardware, the SDK initializes a context successfully but
// device enumeration is empty — all device-dependent paths return "not found"
// or skip the test.

#include <gtest/gtest.h>

#include "vxl_camera/camera_backend.hpp"

#include <atomic>
#include <stdexcept>

using vxl_camera::CameraBackendPtr;
using vxl_camera::makeSdkCameraBackend;

namespace
{

// Try to construct an SdkCameraBackend; skip the test if SDK init fails
// (e.g. libuvc unavailable in the test environment).
CameraBackendPtr tryMake()
{
  try {
    return makeSdkCameraBackend();
  } catch (const std::exception &) {
    return nullptr;
  }
}

}  // namespace

TEST(SdkCameraBackend, ConstructionDoesNotCrash)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP() << "SDK context init unavailable in this environment";}
  EXPECT_FALSE(b->isOpen());
  EXPECT_FALSE(b->isStreaming());
}

TEST(SdkCameraBackend, OpenWithBadSerialReturnsFalse)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  // A serial that cannot match any real device.
  EXPECT_FALSE(b->open("DOES-NOT-EXIST-zzz-9999"));
  EXPECT_FALSE(b->isOpen());
}

TEST(SdkCameraBackend, OpenWithEmptySerialNoDevicesReturnsFalse)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  // If hardware *is* present, this might succeed. We only assert behavior
  // for the no-device case — skip if a device shows up.
  bool opened = b->open("");
  if (opened) {
    GTEST_SKIP() << "A real device is connected; this test only covers no-device case";
  }
  EXPECT_FALSE(b->isOpen());
}

TEST(SdkCameraBackend, GetDeviceInfoBeforeOpenThrows)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_THROW(b->getDeviceInfo(), std::runtime_error);
}

TEST(SdkCameraBackend, HwResetBeforeOpenThrows)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_THROW(b->hwReset(), std::runtime_error);
}

TEST(SdkCameraBackend, IntrinsicsBeforeOpenReturnsNullopt)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_FALSE(b->getIntrinsics(vxl::SensorType::Color).has_value());
  EXPECT_FALSE(b->getIntrinsics(vxl::SensorType::Depth).has_value());
}

TEST(SdkCameraBackend, ExtrinsicsBeforeOpenReturnsNullopt)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_FALSE(
    b->getExtrinsics(vxl::SensorType::Color, vxl::SensorType::Depth).has_value());
}

TEST(SdkCameraBackend, IsOptionSupportedBeforeOpenReturnsFalse)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_FALSE(b->isOptionSupported(vxl::SensorType::Color, VXL_OPTION_EXPOSURE));
}

TEST(SdkCameraBackend, CloseBeforeOpenIsSafe)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_NO_THROW(b->close());
  EXPECT_FALSE(b->isOpen());
}

TEST(SdkCameraBackend, StopStreamingBeforeStartIsSafe)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  EXPECT_NO_THROW(b->stopStreaming());
  EXPECT_FALSE(b->isStreaming());
}

TEST(SdkCameraBackend, SetDeviceEventCallbackAcceptsLambda)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  std::atomic<int> calls{0};
  EXPECT_NO_THROW(
    b->setDeviceEventCallback(
      [&calls](const vxl::DeviceInfo &, bool) {calls++;}));
  // We can't synthesize a hotplug event from outside the SDK, so we only
  // check that the registration call itself doesn't throw.
}

TEST(SdkCameraBackend, SetDeviceEventCallbackNullClearsHandler)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  b->setDeviceEventCallback([](const vxl::DeviceInfo &, bool) {});
  EXPECT_NO_THROW(b->setDeviceEventCallback(nullptr));
}

TEST(SdkCameraBackend, StartStreamingWithoutDeviceThrows)
{
  auto b = tryMake();
  if (!b) {GTEST_SKIP();}
  vxl_camera::BackendStreamConfig cfg;
  cfg.color_enabled = true;
  EXPECT_THROW(
    b->startStreaming(cfg, [](vxl_camera::BackendFrameSetPtr) {}),
    std::runtime_error);
}

TEST(SdkCameraBackend, DestructorCleansUpFromAnyState)
{
  // Multiple lifecycle phases reach destructor — verify each is leak-free.
  {
    auto b = tryMake();
    if (!b) {GTEST_SKIP();}
  }  // never opened
  {
    auto b = tryMake();
    if (!b) {GTEST_SKIP();}
    b->open("DOES-NOT-EXIST");  // failed open
  }
  {
    auto b = tryMake();
    if (!b) {GTEST_SKIP();}
    b->close();
    b->close();  // double close
  }
  // If we got here without crashing, RAII is sound.
  SUCCEED();
}
