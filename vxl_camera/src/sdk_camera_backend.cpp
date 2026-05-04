#include "vxl_camera/camera_backend.hpp"

#include <vxl_filter.hpp>
#include <vxl_dip.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
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
    // Drop cached calibration — a different device may be opened next.
    {
      std::lock_guard<std::mutex> lk(calib_mutex_);
      depth_intrin_cache_.reset();
      color_intrin_cache_.reset();
      depth_to_color_ext_cache_.reset();
    }
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
    // Host-side pipeline (rebuild and swap under mutex; poll thread reads).
    auto pipeline = buildFilterPipeline(chain);
    {
      std::lock_guard<std::mutex> lk(filter_mutex_);
      filter_pipeline_ = std::move(pipeline);
    }

    // Device-side options on the depth sensor (silently no-op if the connected
    // device doesn't expose them, e.g. VXL435 ignores VXL6X5_OPTION_*).
    if (!device_) {return;}
    auto trySet = [this](vxl_option_t opt, float val) {
        try {
          if (isOptionSupported(vxl::SensorType::Depth, opt)) {
            setOption(vxl::SensorType::Depth, opt, val);
          }
        } catch (const std::exception &) {
          // Best-effort — don't break the stream over a single bad option.
        }
      };
    trySet(VXL6X5_OPTION_DENOISE_ENABLE, chain.device.denoise.enabled ? 1.0f : 0.0f);
    if (chain.device.denoise.enabled) {
      trySet(VXL6X5_OPTION_DENOISE_LEVEL, static_cast<float>(chain.device.denoise.level));
    }
    trySet(VXL6X5_OPTION_MEDIAN_FILTER, chain.device.median.enabled ? 1.0f : 0.0f);
    if (chain.device.median.enabled) {
      trySet(VXL6X5_OPTION_MEDIAN_KERNEL_SIZE,
        static_cast<float>(chain.device.median.kernel_size));
    }
    trySet(VXL6X5_OPTION_OUTLIER_REMOVAL,
      chain.device.outlier_removal.enabled ? 1.0f : 0.0f);
  }

  void setAlignDepthToColor(bool enabled, float scale) override
  {
    align_enabled_.store(enabled);
    align_scale_.store(scale);
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
        auto color_frame = frameset->getColorFrame();
        vxl::FramePtr final_depth = raw_depth;
        if (filters && !filters->empty() && raw_depth && raw_depth->isValid()) {
          try {
            auto filtered = filters->process(raw_depth);
            if (filtered) {final_depth = filtered;}
          } catch (const vxl::Error & e) {
            (void)e;  // fall back to raw on failure
          }
        }

        // Optional depth-to-color alignment, on the (possibly filtered) depth.
        vxl::FramePtr aligned_depth;
        if (align_enabled_.load() && final_depth && color_frame &&
          final_depth->isValid() && color_frame->isValid())
        {
          aligned_depth = computeAlignedDepth(final_depth, align_scale_.load());
        }

        auto bs = std::make_shared<BackendFrameSet>();
        bs->color = copyFrame(color_frame);
        bs->depth = copyFrame(final_depth);
        bs->ir = copyFrame(frameset->getIRFrame());
        bs->aligned_depth = copyFrame(aligned_depth);
        callback_(bs);
      } catch (const vxl::Error &) {
        // Timeout or transient error — keep polling.
      }
    }
  }

  // Convert SDK Intrinsics struct to the C-API form used by vxl_dip_*.
  static vxl_intrinsics_t toCIntrinsics(const vxl::Intrinsics & i)
  {
    vxl_intrinsics_t c{};
    c.width = i.width;
    c.height = i.height;
    c.fx = i.fx;
    c.fy = i.fy;
    c.cx = i.cx;
    c.cy = i.cy;
    for (int k = 0; k < 5; k++) {c.coeffs[k] = i.coeffs[k];}
    return c;
  }

  // Convert SDK Extrinsics struct to the C-API form.
  static vxl_extrinsics_t toCExtrinsics(const vxl::Extrinsics & e)
  {
    vxl_extrinsics_t c{};
    for (int k = 0; k < 9; k++) {c.rotation[k] = e.rotation[k];}
    for (int k = 0; k < 3; k++) {c.translation[k] = e.translation[k];}
    return c;
  }

  // Reproject depth into color view via vxl_dip_align_depth_to_rgb. Caches
  // the calibration on first call (queries are not cheap). Returns nullptr
  // on any error so the caller can fall back to raw.
  vxl::FramePtr computeAlignedDepth(const vxl::FramePtr & depth, float scale)
  {
    if (!device_) {return nullptr;}

    // Cache calibration once (intrinsics + extrinsics don't change while open).
    {
      std::lock_guard<std::mutex> lk(calib_mutex_);
      if (!depth_intrin_cache_) {
        try {
          depth_intrin_cache_ = device_->getIntrinsics(vxl::SensorType::Depth);
          color_intrin_cache_ = device_->getIntrinsics(vxl::SensorType::Color);
          depth_to_color_ext_cache_ =
            device_->getExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color);
        } catch (const vxl::Error &) {
          return nullptr;  // calibration unavailable
        }
      }
    }

    vxl_intrinsics_t di = toCIntrinsics(*depth_intrin_cache_);
    vxl_intrinsics_t ci = toCIntrinsics(*color_intrin_cache_);
    vxl_extrinsics_t ext = toCExtrinsics(*depth_to_color_ext_cache_);

    vxl_frame_t * out = nullptr;
    auto err = vxl_dip_align_depth_to_rgb(depth->handle(), &di, &ci, &ext, scale, &out);
    if (err != VXL_SUCCESS || !out) {return nullptr;}
    return vxl::Frame::create(out, /*own=*/ true);
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

  // Depth-to-color alignment.
  std::atomic<bool> align_enabled_{false};
  std::atomic<float> align_scale_{1.0f};
  // Calibration cached on first alignment call to avoid querying every frame.
  std::mutex calib_mutex_;
  std::optional<vxl::Intrinsics> depth_intrin_cache_;
  std::optional<vxl::Intrinsics> color_intrin_cache_;
  std::optional<vxl::Extrinsics> depth_to_color_ext_cache_;
};

}  // namespace

CameraBackendPtr makeSdkCameraBackend()
{
  return std::make_shared<SdkCameraBackend>();
}

}  // namespace vxl_camera
