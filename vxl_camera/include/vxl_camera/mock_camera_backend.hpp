#ifndef VXL_CAMERA__MOCK_CAMERA_BACKEND_HPP_
#define VXL_CAMERA__MOCK_CAMERA_BACKEND_HPP_

#include "vxl_camera/camera_backend.hpp"

#include <atomic>
#include <map>
#include <mutex>
#include <thread>

namespace vxl_camera
{

// In-memory ICameraBackend for tests and downstream consumer mocking.
//
// Usage:
//   auto mock = std::make_shared<MockCameraBackend>();
//   mock->setDeviceInfo({"vxl-test", "SN001", ...});
//   mock->setSupportedOption(SensorType::Color, VXL_OPTION_EXPOSURE, {1, 10000, 1, 5000});
//   auto node = std::make_shared<VxlCameraNode>(opts, mock);  // inject
//
// The mock records every setOption call (see calls_setOption); tests can
// assert on the recorded values to verify parameter routing logic.
class MockCameraBackend : public ICameraBackend
{
public:
  // ── Configuration knobs (set by tests before injecting) ─────────────────
  void setDeviceInfo(const vxl::DeviceInfo & info) {info_ = info;}

  void setIntrinsics(vxl::SensorType s, const vxl::Intrinsics & i) {intrinsics_[s] = i;}

  void setExtrinsics(vxl::SensorType from, vxl::SensorType to, const vxl::Extrinsics & e)
  {
    extrinsics_[{from, to}] = e;
  }

  // Register an option as supported with the given range. After this,
  // isOptionSupported() returns true and getOption()/setOption() work.
  void setSupportedOption(
    vxl::SensorType s, vxl_option_t o, const vxl_option_range_t & range)
  {
    options_[{s, o}] = OptionState{range, range.def};
  }

  // Make open() fail — for testing failure paths.
  void setOpenFails(bool fails) {open_fails_ = fails;}

  // Toggle whether subsequent setOption calls throw (simulates SDK errors).
  void setSetOptionThrows(bool throws) {set_option_throws_ = throws;}

  // ── Test inspection ──────────────────────────────────────────────────────
  // Recorded setOption calls — tests assert on size/contents.
  struct SetOptionCall {
    vxl::SensorType sensor;
    vxl_option_t option;
    float value;
  };

  std::vector<SetOptionCall> calls_setOption() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return set_option_calls_;
  }

  bool didStartStreaming() const {return start_streaming_calls_.load() > 0;}
  bool didStopStreaming() const {return stop_streaming_calls_.load() > 0;}
  int startStreamingCount() const {return start_streaming_calls_.load();}
  int stopStreamingCount() const {return stop_streaming_calls_.load();}
  int hwResetCount() const {return hw_reset_calls_.load();}
  std::optional<BackendStreamConfig> lastStreamConfig() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return last_stream_config_;
  }

  // ── Test driving (manually injecting frames or device events) ───────────
  // Push one frameset to the streaming callback (no-op if not streaming).
  void emitFrameSet(BackendFrameSetPtr fs)
  {
    BackendFrameSetCallback cb;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cb = stream_callback_;
    }
    if (cb && fs) {cb(fs);}
  }

  // Fire a device add/remove event.
  void emitDeviceEvent(const vxl::DeviceInfo & info, bool added)
  {
    BackendDeviceEventCallback cb;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      cb = device_event_callback_;
    }
    if (cb) {cb(info, added);}
  }

  // ── ICameraBackend ──────────────────────────────────────────────────────
  bool open(const std::string & /*serial*/) override
  {
    if (open_fails_) {return false;}
    open_ = true;
    return true;
  }

  void close() override
  {
    stopStreaming();
    open_ = false;
  }

  bool isOpen() const override {return open_;}

  vxl::DeviceInfo getDeviceInfo() const override {return info_;}

  void hwReset() override {hw_reset_calls_++;}

  std::optional<vxl::Intrinsics> getIntrinsics(vxl::SensorType s) const override
  {
    auto it = intrinsics_.find(s);
    if (it == intrinsics_.end()) {return std::nullopt;}
    return it->second;
  }

  std::optional<vxl::Extrinsics> getExtrinsics(
    vxl::SensorType from, vxl::SensorType to) const override
  {
    auto it = extrinsics_.find({from, to});
    if (it == extrinsics_.end()) {return std::nullopt;}
    return it->second;
  }

  bool isOptionSupported(vxl::SensorType s, vxl_option_t o) const override
  {
    return options_.count({s, o}) > 0;
  }

  vxl_option_range_t getOptionRange(vxl::SensorType s, vxl_option_t o) const override
  {
    auto it = options_.find({s, o});
    if (it == options_.end()) {throw std::runtime_error("option not supported");}
    return it->second.range;
  }

  float getOption(vxl::SensorType s, vxl_option_t o) const override
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = options_.find({s, o});
    if (it == options_.end()) {throw std::runtime_error("option not supported");}
    return it->second.value;
  }

  void setOption(vxl::SensorType s, vxl_option_t o, float v) override
  {
    if (set_option_throws_) {throw std::runtime_error("setOption throws (mock)");}
    std::lock_guard<std::mutex> lk(mtx_);
    auto it = options_.find({s, o});
    if (it == options_.end()) {throw std::runtime_error("option not supported");}
    it->second.value = v;
    set_option_calls_.push_back({s, o, v});
  }

  void startStreaming(
    const BackendStreamConfig & config, BackendFrameSetCallback cb) override
  {
    std::lock_guard<std::mutex> lk(mtx_);
    streaming_ = true;
    stream_callback_ = std::move(cb);
    last_stream_config_ = config;
    start_streaming_calls_++;
  }

  void stopStreaming() override
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!streaming_) {return;}
    streaming_ = false;
    stream_callback_ = nullptr;
    stop_streaming_calls_++;
  }

  bool isStreaming() const override
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return streaming_;
  }

  void setDeviceEventCallback(BackendDeviceEventCallback cb) override
  {
    std::lock_guard<std::mutex> lk(mtx_);
    device_event_callback_ = std::move(cb);
  }

private:
  struct OptionState {
    vxl_option_range_t range;
    float value;
  };

  // Composite key for sensor+option lookup.
  using OptionKey = std::pair<vxl::SensorType, vxl_option_t>;
  using ExtrinsicsKey = std::pair<vxl::SensorType, vxl::SensorType>;

  vxl::DeviceInfo info_;
  std::map<vxl::SensorType, vxl::Intrinsics> intrinsics_;
  std::map<ExtrinsicsKey, vxl::Extrinsics> extrinsics_;
  std::map<OptionKey, OptionState> options_;

  bool open_ = false;
  bool open_fails_ = false;
  bool set_option_throws_ = false;

  // Streaming state
  bool streaming_ = false;
  BackendFrameSetCallback stream_callback_;
  std::optional<BackendStreamConfig> last_stream_config_;
  std::atomic<int> start_streaming_calls_{0};
  std::atomic<int> stop_streaming_calls_{0};
  std::atomic<int> hw_reset_calls_{0};
  std::vector<SetOptionCall> set_option_calls_;
  BackendDeviceEventCallback device_event_callback_;

  mutable std::mutex mtx_;
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__MOCK_CAMERA_BACKEND_HPP_
