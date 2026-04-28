#include "vxl_camera/camera_backend.hpp"

#include <vxl_filter.hpp>

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace vxl_camera
{

namespace
{

// Convert an SDK frame to a self-contained BackendFrame. Copies the pixel
// buffer so the BackendFrame can outlive the SDK frame.
BackendFramePtr copyFrame(const vxl::FramePtr & frame)
{
  if (!frame || !frame->isValid()) {return nullptr;}
  auto bf = std::make_shared<BackendFrame>();
  bf->width = frame->width();
  bf->height = frame->height();
  bf->stride = frame->stride();
  bf->format = frame->format();
  size_t n = frame->dataSize();
  bf->data.resize(n);
  std::memcpy(bf->data.data(), frame->data(), n);
  auto m = frame->metadata();
  bf->timestamp_us = m.timestamp_us;
  bf->sequence = m.sequence;
  bf->exposure_us = m.exposure_us;
  bf->gain = m.gain;
  return bf;
}

class SdkCameraBackend : public ICameraBackend
{
public:
  SdkCameraBackend() {context_ = vxl::Context::create();}
  ~SdkCameraBackend() override {close();}

  bool open(const std::string & serial) override
  {
    if (serial.empty()) {
      if (context_->deviceCount() == 0) {return false;}
      device_ = context_->getDevice(0);
    } else {
      device_ = context_->findDeviceBySerial(serial);
      if (!device_) {return false;}
    }
    device_->open();
    return true;
  }

  void close() override
  {
    stopStreaming();
    if (device_ && device_->isOpen()) {device_->close();}
    device_.reset();
    // Keep context_ alive so device events keep firing during reconnect attempts.
  }

  bool isOpen() const override
  {
    return device_ && device_->isOpen();
  }

  vxl::DeviceInfo getDeviceInfo() const override
  {
    if (!device_) {throw std::runtime_error("device not open");}
    return device_->getInfo();
  }

  void hwReset() override
  {
    if (!device_) {throw std::runtime_error("device not open");}
    device_->hwReset();
  }

  std::optional<vxl::Intrinsics> getIntrinsics(vxl::SensorType s) const override
  {
    if (!device_) {return std::nullopt;}
    try {
      return device_->getIntrinsics(s);
    } catch (const vxl::Error &) {
      return std::nullopt;
    }
  }

  std::optional<vxl::Extrinsics> getExtrinsics(
    vxl::SensorType from, vxl::SensorType to) const override
  {
    if (!device_) {return std::nullopt;}
    try {
      return device_->getExtrinsics(from, to);
    } catch (const vxl::Error &) {
      return std::nullopt;
    }
  }

  bool isOptionSupported(vxl::SensorType s, vxl_option_t o) const override
  {
    if (!device_) {return false;}
    try {
      auto sensor = device_->getSensor(s);
      return sensor && sensor->isOptionSupported(o);
    } catch (const vxl::Error &) {
      return false;
    }
  }

  vxl_option_range_t getOptionRange(vxl::SensorType s, vxl_option_t o) const override
  {
    auto sensor = device_->getSensor(s);
    return sensor->getOptionRange(o);
  }

  float getOption(vxl::SensorType s, vxl_option_t o) const override
  {
    auto sensor = device_->getSensor(s);
    return sensor->getOption(o);
  }

  void setOption(vxl::SensorType s, vxl_option_t o, float v) override
  {
    auto sensor = device_->getSensor(s);
    sensor->setOption(o, v);
  }

  void startStreaming(
    const BackendStreamConfig & config, BackendFrameSetCallback cb) override
  {
    if (running_.load()) {return;}
    if (!device_) {throw std::runtime_error("device not open");}
    callback_ = std::move(cb);

    pipeline_ = std::make_unique<vxl::Pipeline>(device_);

    // Sync mode
    if (config.sync_mode == "strict") {
      pipeline_->setSyncMode(vxl::SyncMode::Strict);
    } else if (config.sync_mode == "approximate") {
      pipeline_->setSyncMode(vxl::SyncMode::Approximate);
    } else {
      pipeline_->setSyncMode(vxl::SyncMode::None);
    }
    pipeline_->setFrameQueueSize(config.frame_queue_size);

    if (config.color_enabled) {
      pipeline_->enableStream(vxl::SensorType::Color, vxl::Format::BGR,
        config.color_width, config.color_height, config.color_fps);
    }
    if (config.depth_enabled) {
      pipeline_->enableStream(vxl::SensorType::Depth, vxl::Format::Z16,
        config.depth_width, config.depth_height, config.depth_fps);
    }
    if (config.ir_enabled) {
      pipeline_->enableStream(vxl::SensorType::IR, vxl::Format::Gray16,
        config.ir_width, config.ir_height, config.ir_fps);
    }

    pipeline_->start();
    running_.store(true);
    try {
      poll_thread_ = std::thread([this]() {pollLoop();});
    } catch (...) {
      running_.store(false);
      pipeline_->stop();
      pipeline_.reset();
      throw;
    }
  }

  void stopStreaming() override
  {
    running_.store(false);
    if (poll_thread_.joinable()) {poll_thread_.join();}
    if (pipeline_ && pipeline_->isRunning()) {pipeline_->stop();}
    pipeline_.reset();
    callback_ = nullptr;
  }

  bool isStreaming() const override
  {
    return running_.load();
  }

  void setDeviceEventCallback(BackendDeviceEventCallback cb) override
  {
    if (!context_) {return;}
    if (!cb) {
      context_->setDeviceEventCallback(nullptr);
      return;
    }
    // Wrap in a stable lambda; the SDK keeps a copy.
    context_->setDeviceEventCallback(
      [cb = std::move(cb)](const vxl::DeviceInfo & info, bool added) {cb(info, added);});
  }

  void setFilterChain(const FilterChain & chain) override
  {
    auto pipeline = buildFilterPipeline(chain);
    std::lock_guard<std::mutex> lk(filter_mutex_);
    filter_pipeline_ = std::move(pipeline);
  }

private:
  void pollLoop()
  {
    while (running_.load()) {
      try {
        auto frameset = pipeline_->waitForFrameSet(100);
        if (!frameset || frameset->empty() || !callback_) {continue;}

        // Apply host-side filter chain to depth frame, if configured.
        // The pipeline is rebuilt on setFilterChain() so we just need a
        // shared_ptr snapshot under the mutex (FilterPipeline itself is
        // not thread-safe to use from multiple threads concurrently, but
        // we only ever apply it from this single poll thread).
        std::shared_ptr<vxl::FilterPipeline> filters;
        {
          std::lock_guard<std::mutex> lk(filter_mutex_);
          filters = filter_pipeline_;
        }

        auto raw_depth = frameset->getDepthFrame();
        vxl::FramePtr final_depth = raw_depth;
        if (filters && !filters->empty() && raw_depth && raw_depth->isValid()) {
          try {
            auto filtered = filters->process(raw_depth);
            if (filtered) {final_depth = filtered;}
          } catch (const vxl::Error & e) {
            // Filter failure should not stop the stream — fall back to raw.
            (void)e;
          }
        }

        auto bs = std::make_shared<BackendFrameSet>();
        bs->color = copyFrame(frameset->getColorFrame());
        bs->depth = copyFrame(final_depth);
        bs->ir = copyFrame(frameset->getIRFrame());
        callback_(bs);
      } catch (const vxl::Error &) {
        // Timeout or transient error — keep polling.
      }
    }
  }

  // Translate a FilterChain config into a vxl::FilterPipeline.
  // Returns nullptr if no filters are enabled (callers can short-circuit).
  static std::shared_ptr<vxl::FilterPipeline> buildFilterPipeline(const FilterChain & fc)
  {
    auto pipe = std::make_shared<vxl::FilterPipeline>();

    // Order matters: decimate first (cheaper downstream), then range-clip,
    // then smooth, then fill holes. Mirrors RealSense recommended order.
    if (fc.decimation.enabled) {
      auto f = std::make_shared<vxl::DecimationFilter>();
      f->setScale(fc.decimation.scale);
      pipe->add(f);
    }
    if (fc.threshold.enabled) {
      auto f = std::make_shared<vxl::ThresholdFilter>();
      f->setMinDistance(fc.threshold.min_mm);
      f->setMaxDistance(fc.threshold.max_mm);
      pipe->add(f);
    }
    if (fc.spatial.enabled) {
      auto f = std::make_shared<vxl::SpatialFilter>();
      f->setMagnitude(fc.spatial.magnitude);
      f->setAlpha(fc.spatial.alpha);
      f->setDelta(fc.spatial.delta);
      pipe->add(f);
    }
    if (fc.temporal.enabled) {
      auto f = std::make_shared<vxl::TemporalFilter>();
      f->setAlpha(fc.temporal.alpha);
      f->setDelta(fc.temporal.delta);
      pipe->add(f);
    }
    if (fc.hole_filling.enabled) {
      auto f = std::make_shared<vxl::HoleFillingFilter>();
      using HM = vxl::HoleFillingFilter::Mode;
      HM m = HM::NearestFromAround;
      switch (fc.hole_filling.mode) {
        case 1: m = HM::FarestFromAround; break;
        case 2: m = HM::FillFromLeft; break;
        default: m = HM::NearestFromAround; break;
      }
      f->setMode(m);
      pipe->add(f);
    }

    return pipe->empty() ? nullptr : pipe;
  }

  vxl::ContextPtr context_;
  vxl::DevicePtr device_;
  std::unique_ptr<vxl::Pipeline> pipeline_;
  BackendFrameSetCallback callback_;
  std::thread poll_thread_;
  std::atomic<bool> running_{false};

  // Filter pipeline (host-side post-processing applied in pollLoop).
  // shared_ptr so the poll thread can take a stable snapshot without
  // holding the mutex for the duration of process().
  std::mutex filter_mutex_;
  std::shared_ptr<vxl::FilterPipeline> filter_pipeline_;
};

}  // namespace

CameraBackendPtr makeSdkCameraBackend()
{
  return std::make_shared<SdkCameraBackend>();
}

}  // namespace vxl_camera
