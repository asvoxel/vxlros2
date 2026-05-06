#ifndef VXL_CAMERA__CAMERA_BACKEND_HPP_
#define VXL_CAMERA__CAMERA_BACKEND_HPP_

#include "vxl_camera/filter_chain.hpp"

#include <vxl.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

// Abstraction over the ASVXL SDK so node logic can be unit-tested without
// real hardware. The SdkCameraBackend implementation wraps Context+Device+
// Pipeline+StreamManager; MockCameraBackend provides controllable in-memory
// behavior for tests.
namespace vxl_camera
{

// Self-contained frame: owns its data buffer plus the SDK metadata fields.
// We don't expose vxl::FramePtr through the interface so mock tests don't
// need to construct SDK frames.
struct BackendFrame {
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t stride = 0;
  vxl::Format format = vxl::Format::Unknown;
  std::vector<uint8_t> data;
  // Frame metadata (mirrors vxl_frame_metadata_t)
  uint64_t timestamp_us = 0;
  uint32_t sequence = 0;
  uint32_t exposure_us = 0;
  uint16_t gain = 0;

  bool isValid() const {return !data.empty();}
};

using BackendFramePtr = std::shared_ptr<BackendFrame>;

struct BackendFrameSet {
  BackendFramePtr color;
  BackendFramePtr depth;
  BackendFramePtr ir;
  // Set only when align_depth_to_color is enabled on the backend; this is the
  // depth frame reprojected into the color camera's view (same resolution as
  // color, with depth values resampled to color pixels).
  BackendFramePtr aligned_depth;

  bool empty() const {return !color && !depth && !ir;}
};

using BackendFrameSetPtr = std::shared_ptr<BackendFrameSet>;
using BackendFrameSetCallback = std::function<void(BackendFrameSetPtr)>;
using BackendDeviceEventCallback = std::function<void(const vxl::DeviceInfo &, bool added)>;

struct BackendStreamConfig {
  bool color_enabled = false;
  bool depth_enabled = false;
  bool ir_enabled = false;
  uint32_t color_width = 1280, color_height = 720, color_fps = 30;
  uint32_t depth_width = 640,  depth_height = 480, depth_fps = 30;
  uint32_t ir_width = 640,     ir_height = 480,    ir_fps = 30;
  std::string sync_mode = "strict";  // strict | approximate | none
  size_t frame_queue_size = 4;
  // Settle delay between successive stream start() calls. The SDK v4l2
  // backend's per-stream STREAMON ioctl submits isoch (Y16) and bulk (MJPEG)
  // URBs into kernel uvcvideo; back-to-back STREAMONs without a settle delay
  // leave the second stream's callback never firing on some hosts (notably
  // VM USB-passthrough). 1000 ms is what the SDK's `dst` reference program
  // uses on physical Linux; 200 ms tends to suffice. Set to 0 to disable
  // (only safe on hardware you've verified single-shot).
  uint32_t inter_stream_start_delay_ms = 1000;
};

// All SDK touchpoints the nodes need. Each method that talks to the device
// throws std::runtime_error (or vxl::Error) on failure; callers handle errors.
class ICameraBackend
{
public:
  virtual ~ICameraBackend() = default;

  // ── Discovery + open ────────────────────────────────────────────────────
  // Open the device matching `serial`, or auto-select the first one if empty.
  // Returns true on success. After open(), getDeviceInfo() and the rest are valid.
  virtual bool open(const std::string & serial = "") = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;
  virtual vxl::DeviceInfo getDeviceInfo() const = 0;
  virtual void hwReset() = 0;

  // ── Calibration ─────────────────────────────────────────────────────────
  // Returns nullopt if the sensor is unavailable or not calibrated.
  virtual std::optional<vxl::Intrinsics> getIntrinsics(vxl::SensorType) const = 0;
  virtual std::optional<vxl::Extrinsics> getExtrinsics(
    vxl::SensorType from, vxl::SensorType to) const = 0;

  // ── Sensor options ──────────────────────────────────────────────────────
  virtual bool isOptionSupported(vxl::SensorType, vxl_option_t) const = 0;
  virtual vxl_option_range_t getOptionRange(vxl::SensorType, vxl_option_t) const = 0;
  virtual float getOption(vxl::SensorType, vxl_option_t) const = 0;
  virtual void setOption(vxl::SensorType, vxl_option_t, float value) = 0;

  // ── Streaming ───────────────────────────────────────────────────────────
  // startStreaming sets up the configured streams and arranges for `cb` to be
  // invoked on each synchronized frameset. The callback may run on a backend-
  // owned thread, so callers must be thread-safe.
  virtual void startStreaming(
    const BackendStreamConfig & config,
    BackendFrameSetCallback cb) = 0;
  virtual void stopStreaming() = 0;
  virtual bool isStreaming() const = 0;

  // ── Hotplug ─────────────────────────────────────────────────────────────
  // Register a callback fired when any device is added or removed at the
  // SDK/USB layer. The callback may run on an SDK-owned thread.
  virtual void setDeviceEventCallback(BackendDeviceEventCallback cb) = 0;

  // ── Depth post-processing ───────────────────────────────────────────────
  // Configure the host-side filter chain applied to depth frames before
  // delivery. Safe to call any time (also while streaming); takes effect
  // from the next frame. Default state is all filters disabled.
  virtual void setFilterChain(const FilterChain & chain) = 0;

  // ── Depth-to-color alignment ────────────────────────────────────────────
  // When enabled, the backend reprojects the (possibly filtered) depth frame
  // into the color camera's view and delivers it as BackendFrameSet::aligned_depth
  // (in addition to the raw depth). `scale` is the device-specific factor that
  // converts raw depth values to millimeters (VXL435=1.0, VXL6X5=8.0, VXL605=16.0).
  // Default disabled.
  virtual void setAlignDepthToColor(bool enabled, float scale) = 0;
};

using CameraBackendPtr = std::shared_ptr<ICameraBackend>;

// Factory for the production backend wired to the real ASVXL SDK.
CameraBackendPtr makeSdkCameraBackend();

}  // namespace vxl_camera

#endif  // VXL_CAMERA__CAMERA_BACKEND_HPP_
