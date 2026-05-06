#include "vxl_camera/camera_backend.hpp"

#include <vxl_filter.hpp>
#include <vxl_dip.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
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

  // ──────────────────────────────────────────────────────────────────────────
  // Lifecycle
  // ──────────────────────────────────────────────────────────────────────────

  bool open(const std::string & serial) override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
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
    stopStreaming();  // joins publish_thread; must happen before sdk_mutex_ acquire
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    if (device_ && device_->isOpen()) {device_->close();}
    device_.reset();
    {
      std::lock_guard<std::mutex> clk(calib_mutex_);
      depth_intrin_cache_.reset();
      color_intrin_cache_.reset();
      depth_to_color_ext_cache_.reset();
    }
    // Keep context_ alive so device events keep firing during reconnect attempts.
  }

  bool isOpen() const override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    return device_ && device_->isOpen();
  }

  vxl::DeviceInfo getDeviceInfo() const override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    if (!device_) {throw std::runtime_error("device not open");}
    return device_->getInfo();
  }

  void hwReset() override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    if (!device_) {throw std::runtime_error("device not open");}
    device_->hwReset();
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Calibration
  // ──────────────────────────────────────────────────────────────────────────

  std::optional<vxl::Intrinsics> getIntrinsics(vxl::SensorType s) const override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
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
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    if (!device_) {return std::nullopt;}
    try {
      return device_->getExtrinsics(from, to);
    } catch (const vxl::Error &) {
      return std::nullopt;
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Sensor options
  // ──────────────────────────────────────────────────────────────────────────

  bool isOptionSupported(vxl::SensorType s, vxl_option_t o) const override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
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
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    auto sensor = device_->getSensor(s);
    return sensor->getOptionRange(o);
  }

  float getOption(vxl::SensorType s, vxl_option_t o) const override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    auto sensor = device_->getSensor(s);
    return sensor->getOption(o);
  }

  void setOption(vxl::SensorType s, vxl_option_t o, float v) override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    auto sensor = device_->getSensor(s);
    sensor->setOption(o, v);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Streaming — direct sensor/stream API, per-stream latest-frame slot,
  // single publish_thread emits BackendFrameSet.
  //
  // Architecture (3-layer "lean producer-consumer"):
  //   L1: SDK backend worker thread (one per physical USB stream) →
  //       stream callback writes the latest FramePtr into our slot (O(1)).
  //   L2: publish_thread waits on cv, snapshots all slots, runs heavy work
  //       (MJPEG→BGR convert, filter chain, depth-to-color align, copyFrame),
  //       invokes user callback (BackendFrameSet ready).
  //   L3: ROS publish (in user callback context — outside sdk_mutex_).
  //
  // Why not vxl::Pipeline:
  //   Pipeline's frame_queue + sync_thread + frameset_queue + outer poll
  //   added 3 extra threading hops between SDK and ROS publish. In real-time
  //   robotics (obstacle avoidance, grasping) latency jitter from those hops
  //   compounds with VM USB-passthrough scheduling sensitivity. Direct sensor
  //   + per-stream slot is the same pattern Orbbec/RealSense use.
  // ──────────────────────────────────────────────────────────────────────────

  void startStreaming(
    const BackendStreamConfig & config, BackendFrameSetCallback cb) override
  {
    {
      std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
      if (running_.load()) {return;}
      if (!device_) {throw std::runtime_error("device not open");}
      callback_ = std::move(cb);
      sync_mode_ = config.sync_mode;
      need_depth_.store(config.depth_enabled);
      need_ir_.store(config.ir_enabled);
      need_color_.store(config.color_enabled);

      // Reset slots from any prior session.
      clearSlot(slot_depth_);
      clearSlot(slot_ir_);
      clearSlot(slot_color_);

      // Stream creation order: Depth → IR → Color.
      // On VXL6X5 the device firmware needs the Y16 stream open (which sets
      // NIR_DEPTH XU mode as a side effect) before the MJPEG bulk pipe will
      // deliver frames; reverse order leaves Color stuck at 0 frames in
      // dual-stream mode.
      try {
        if (config.depth_enabled) {
          openStreamAndStore(
            stream_depth_, sensor_depth_,
            vxl::SensorType::Depth, vxl::Format::Z16,
            config.depth_width, config.depth_height, config.depth_fps);
        }
        if (config.ir_enabled) {
          openStreamAndStore(
            stream_ir_, sensor_ir_,
            vxl::SensorType::IR, vxl::Format::Gray16,
            config.ir_width, config.ir_height, config.ir_fps);
        }
        if (config.color_enabled) {
          // Format::Any lets the SDK pick the device's native profile (e.g.
          // MJPEG on VXL615) rather than requiring a Profile{BGR, w, h, fps};
          // we convert to BGR in the publish thread before the user callback.
          openStreamAndStore(
            stream_color_, sensor_color_,
            vxl::SensorType::Color, vxl::Format::Any,
            config.color_width, config.color_height, config.color_fps);
        }

        running_.store(true);

        // Start streams in the same Depth→IR→Color order, with a settle
        // delay between successive stream->start() calls.
        //
        // Empirically: the SDK v4l2 backend's per-stream STREAMON ioctl
        // submits isoch (Y16) and bulk (MJPEG) URBs into kernel uvcvideo;
        // back-to-back STREAMONs without a settle delay leave the second
        // stream with kernel buffers that never fill (DQBUF returns nothing).
        // dst in our matrix tests uses sleep(1) between stream_starts and
        // gets 102+173 frames; sdk_camera_backend without a settle gets
        // 0+0. 200ms is enough on the hardware we tested.
        auto start_with_settle =
          [](vxl::StreamPtr & s, auto cb, bool more_to_start) {
            if (!s) {return;}
            s->start(std::move(cb));
            if (more_to_start) {
              std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
          };

        const bool has_ir    = stream_ir_    != nullptr;
        const bool has_color = stream_color_ != nullptr;

        start_with_settle(
          stream_depth_,
          [this](vxl::FramePtr f) {onStreamFrame(SlotKind::Depth, std::move(f));},
          has_ir || has_color);
        start_with_settle(
          stream_ir_,
          [this](vxl::FramePtr f) {onStreamFrame(SlotKind::IR, std::move(f));},
          has_color);
        start_with_settle(
          stream_color_,
          [this](vxl::FramePtr f) {onStreamFrame(SlotKind::Color, std::move(f));},
          false);
      } catch (...) {
        running_.store(false);
        teardownStreamsLocked();
        throw;
      }
    }  // release sdk_mutex_ before spawning publish_thread

    try {
      publish_thread_ = std::thread([this]() {publishLoop();});
    } catch (...) {
      stopStreaming();
      throw;
    }
  }

  void stopStreaming() override
  {
    // Signal stop and wake publish_thread so it can exit promptly.
    running_.store(false);
    publish_cv_.notify_all();

    // Join publish_thread BEFORE acquiring sdk_mutex_ — the publish loop
    // acquires sdk_mutex_ for frame processing; holding it here would
    // deadlock against an in-flight iteration.
    if (publish_thread_.joinable()) {publish_thread_.join();}

    // Stop streams under sdk_mutex_. SDK guarantees stop() drains in-flight
    // backend callbacks before returning, so no further onStreamFrame races.
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    teardownStreamsLocked();
    clearSlot(slot_depth_);
    clearSlot(slot_ir_);
    clearSlot(slot_color_);
    callback_ = nullptr;
  }

  bool isStreaming() const override
  {
    return running_.load();
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Hotplug, filter chain, alignment
  // ──────────────────────────────────────────────────────────────────────────

  void setDeviceEventCallback(BackendDeviceEventCallback cb) override
  {
    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
    if (!context_) {return;}
    if (!cb) {
      context_->setDeviceEventCallback(nullptr);
      return;
    }
    // Wrap in a stable lambda; the SDK keeps a copy. The lambda is invoked
    // from the SDK's hotplug event thread — it must NOT call back into SDK
    // (would race against the user-thread's sdk_mutex_-protected calls).
    // We just forward to the user callback which is expected to do its own
    // locking/queueing.
    context_->setDeviceEventCallback(
      [cb = std::move(cb)](const vxl::DeviceInfo & info, bool added) {cb(info, added);});
  }

  void setFilterChain(const FilterChain & chain) override
  {
    auto pipeline = buildFilterPipeline(chain);
    {
      std::lock_guard<std::mutex> lk(filter_mutex_);
      filter_pipeline_ = std::move(pipeline);
    }

    std::lock_guard<std::recursive_mutex> lk(sdk_mutex_);
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
  // ──────────────────────────────────────────────────────────────────────────
  // Internal: per-stream slot
  //
  // Each enabled stream gets a single-frame slot. SDK callbacks write into
  // it (replacing any prior unconsumed frame — natural drop-oldest backpressure,
  // bounded memory). publish_thread snapshots all slots and decides whether
  // to emit a BackendFrameSet based on sync_mode_.
  // ──────────────────────────────────────────────────────────────────────────

  enum class SlotKind { Depth, IR, Color };

  struct StreamSlot {
    std::mutex mu;
    vxl::FramePtr latest;
    bool fresh = false;            // true if a new frame arrived since last emit
    uint64_t timestamp_us = 0;
  };

  StreamSlot & slotFor(SlotKind k)
  {
    switch (k) {
      case SlotKind::Depth: return slot_depth_;
      case SlotKind::IR:    return slot_ir_;
      case SlotKind::Color: return slot_color_;
    }
    return slot_depth_;  // unreachable
  }

  static void clearSlot(StreamSlot & s)
  {
    std::lock_guard<std::mutex> lk(s.mu);
    s.latest.reset();
    s.fresh = false;
    s.timestamp_us = 0;
  }

  // Hot path — runs on SDK backend worker thread. Keep this tiny: write the
  // latest pointer + notify the publish thread. NO heavy work, NO sdk_mutex_.
  void onStreamFrame(SlotKind kind, vxl::FramePtr frame)
  {
    if (!frame || !frame->isValid()) {return;}
    auto ts = frame->metadata().timestamp_us;
    StreamSlot & s = slotFor(kind);
    {
      std::lock_guard<std::mutex> lk(s.mu);
      s.latest = std::move(frame);
      s.fresh = true;
      s.timestamp_us = ts;
    }
    publish_cv_.notify_one();
  }

  // Open a stream and store BOTH the sensor and the stream — the SDK's
  // vxl::Stream internally references the sensor's lifetime, so dropping
  // the sensor shared_ptr after createStream returns leads to use-after-free
  // inside stream->start (manifests as bad-mutex segfault in v4l2 backend
  // dev->lock acquisition). Mirror the vxl::Pipeline pattern that retains
  // both in parallel maps.
  void openStreamAndStore(
    vxl::StreamPtr & out_stream, vxl::SensorPtr & out_sensor,
    vxl::SensorType type, vxl::Format fmt, uint32_t w, uint32_t h, uint32_t fps)
  {
    auto sensor = device_->getSensor(type);
    if (!sensor) {throw std::runtime_error("no sensor for requested type");}
    auto profile = sensor->findProfile(fmt, w, h, fps);
    if (!profile) {
      throw std::runtime_error(
              "no matching profile (format=" + std::to_string(static_cast<int>(fmt)) +
              ", " + std::to_string(w) + "x" + std::to_string(h) + "@" +
              std::to_string(fps) + "fps)");
    }
    out_stream = sensor->createStream(profile);
    out_sensor = std::move(sensor);  // KEEP sensor alive for stream's lifetime
  }

  // Caller must hold sdk_mutex_.
  void teardownStreamsLocked()
  {
    auto safeStop = [](vxl::StreamPtr & s) {
        if (!s) {return;}
        try {s->stop();} catch (...) {}
        s.reset();
      };
    // Reverse order from start (Color → IR → Depth) so MJPEG bulk releases
    // before Y16 isoch — matches SDK Pipeline teardown convention.
    safeStop(stream_color_);
    safeStop(stream_ir_);
    safeStop(stream_depth_);
    // Now drop sensors — stream destruction must happen first (stream
    // internally references sensor state).
    sensor_color_.reset();
    sensor_ir_.reset();
    sensor_depth_.reset();
  }

  // ──────────────────────────────────────────────────────────────────────────
  // publish_thread main loop
  //
  // Wait on cv → snapshot slots → decide emit (per sync_mode_) → process
  // (convert/filter/align/copyFrame under sdk_mutex_) → user callback
  // (outside sdk_mutex_).
  //
  // The 100ms timeout on cv.wait_for ensures we wake even if no callback
  // arrives (e.g. USB stall) — lets us re-check running_ and exit cleanly.
  // ──────────────────────────────────────────────────────────────────────────

  void publishLoop()
  {
    while (running_.load()) {
      // Wait for a stream callback to notify us, or 100ms timeout for shutdown.
      {
        std::unique_lock<std::mutex> lk(publish_mu_);
        publish_cv_.wait_for(lk, std::chrono::milliseconds(100));
      }
      if (!running_.load()) {break;}

      const bool need_d = need_depth_.load();
      const bool need_i = need_ir_.load();
      const bool need_c = need_color_.load();

      // Snapshot slot state under each slot's own mutex. Capturing the
      // FramePtr by copy keeps the data alive even if the SDK callback
      // replaces the slot.latest before we finish processing.
      vxl::FramePtr f_d, f_i, f_c;
      uint64_t ts_d = 0, ts_i = 0, ts_c = 0;
      bool fresh_d = false, fresh_i = false, fresh_c = false;
      auto snap = [](StreamSlot & s, vxl::FramePtr & out, uint64_t & ts, bool & fresh) {
          std::lock_guard<std::mutex> lk(s.mu);
          out = s.latest;
          ts = s.timestamp_us;
          fresh = s.fresh;
        };
      if (need_d) {snap(slot_depth_, f_d, ts_d, fresh_d);}
      if (need_i) {snap(slot_ir_,    f_i, ts_i, fresh_i);}
      if (need_c) {snap(slot_color_, f_c, ts_c, fresh_c);}

      // Decide whether to emit a BackendFrameSet.
      //
      // - none:        emit when ANY enabled stream has a fresh frame
      //                (others contribute their latest if present, or null)
      // - approximate: emit when ALL enabled streams have fresh frames
      //                (no timestamp tolerance check)
      // - strict:      same as approximate AND all timestamps within tolerance
      //                (default 16ms = half a 30fps frame period). If
      //                tolerance fails, drop the fresh marks and wait — next
      //                callback will replace with newer frames.
      bool should_emit = false;
      if (sync_mode_ == "none") {
        should_emit =
          (need_d && fresh_d) || (need_i && fresh_i) || (need_c && fresh_c);
      } else {
        const bool all_fresh =
          (!need_d || fresh_d) && (!need_i || fresh_i) && (!need_c || fresh_c);
        should_emit = all_fresh;
        if (should_emit && sync_mode_ == "strict") {
          uint64_t min_ts = UINT64_MAX, max_ts = 0;
          if (need_d) {min_ts = std::min(min_ts, ts_d); max_ts = std::max(max_ts, ts_d);}
          if (need_i) {min_ts = std::min(min_ts, ts_i); max_ts = std::max(max_ts, ts_i);}
          if (need_c) {min_ts = std::min(min_ts, ts_c); max_ts = std::max(max_ts, ts_c);}
          constexpr uint64_t TOL_US = 16000;  // half a frame @30fps
          if (max_ts - min_ts > TOL_US) {should_emit = false;}
        }
      }
      if (!should_emit) {continue;}

      // Mark consumed (drop the fresh flag so we wait for the next
      // callback before emitting again — prevents emitting the same pair
      // repeatedly).
      consumeFresh(slot_depth_, fresh_d);
      consumeFresh(slot_ir_,    fresh_i);
      consumeFresh(slot_color_, fresh_c);

      // Process frames + assemble BackendFrameSet under sdk_mutex_.
      // sdk_mutex_ guards SDK API access (frame->format/convert,
      // filter pipeline process, vxl_dip_align_depth_to_rgb).
      BackendFrameSetPtr bs;
      {
        std::lock_guard<std::recursive_mutex> sdk_lk(sdk_mutex_);

        // Color: MJPEG/YUYV → BGR if needed (ROS image_raw expects bgr8 etc.)
        if (f_c && f_c->isValid()) {
          auto fmt = f_c->format();
          if (fmt != vxl::Format::BGR && fmt != vxl::Format::RGB &&
            fmt != vxl::Format::Gray8 && fmt != vxl::Format::Gray16)
          {
            try {
              auto converted = f_c->convert(vxl::Format::BGR);
              if (converted && converted->isValid()) {f_c = converted;}
            } catch (const vxl::Error &) {
              f_c.reset();  // drop the unconvertible color frame
            }
          }
        }

        // Depth: optional host-side filter chain.
        std::shared_ptr<vxl::FilterPipeline> filters;
        {
          std::lock_guard<std::mutex> flk(filter_mutex_);
          filters = filter_pipeline_;
        }
        vxl::FramePtr final_depth = f_d;
        if (filters && !filters->empty() && f_d && f_d->isValid()) {
          try {
            auto out = filters->process(f_d);
            if (out) {final_depth = out;}
          } catch (const vxl::Error &) {/* fall back to raw */}
        }

        // Optional depth-to-color alignment.
        vxl::FramePtr aligned;
        if (align_enabled_.load() && final_depth && f_c &&
          final_depth->isValid() && f_c->isValid())
        {
          aligned = computeAlignedDepth(final_depth, align_scale_.load());
        }

        bs = std::make_shared<BackendFrameSet>();
        bs->color         = copyFrame(f_c);
        bs->depth         = copyFrame(final_depth);
        bs->ir            = copyFrame(f_i);
        bs->aligned_depth = copyFrame(aligned);
      }  // release sdk_mutex_ before user callback

      // User callback runs OUTSIDE sdk_mutex_:
      //   - frees the SDK for parallel option queries from main thread
      //   - allows the user callback to call setOption etc. without deadlock
      //   - bs already owns copies of the frame data, no SDK reference left
      if (callback_) {callback_(bs);}
    }
  }

  static void consumeFresh(StreamSlot & s, bool was_fresh)
  {
    if (!was_fresh) {return;}
    std::lock_guard<std::mutex> lk(s.mu);
    s.fresh = false;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Calibration helpers (depth-to-color alignment)
  // ──────────────────────────────────────────────────────────────────────────

  static vxl_intrinsics_t toCIntrinsics(const vxl::Intrinsics & i)
  {
    vxl_intrinsics_t c{};
    c.width = i.width; c.height = i.height;
    c.fx = i.fx; c.fy = i.fy; c.cx = i.cx; c.cy = i.cy;
    for (int k = 0; k < 5; k++) {c.coeffs[k] = i.coeffs[k];}
    return c;
  }

  static vxl_extrinsics_t toCExtrinsics(const vxl::Extrinsics & e)
  {
    vxl_extrinsics_t c{};
    for (int k = 0; k < 9; k++) {c.rotation[k] = e.rotation[k];}
    for (int k = 0; k < 3; k++) {c.translation[k] = e.translation[k];}
    return c;
  }

  // Caller must hold sdk_mutex_ (we call device_->getIntrinsics for cache fill).
  vxl::FramePtr computeAlignedDepth(const vxl::FramePtr & depth, float scale)
  {
    if (!device_) {return nullptr;}
    {
      std::lock_guard<std::mutex> lk(calib_mutex_);
      if (!depth_intrin_cache_) {
        try {
          depth_intrin_cache_ = device_->getIntrinsics(vxl::SensorType::Depth);
          color_intrin_cache_ = device_->getIntrinsics(vxl::SensorType::Color);
          depth_to_color_ext_cache_ =
            device_->getExtrinsics(vxl::SensorType::Depth, vxl::SensorType::Color);
        } catch (const vxl::Error &) {
          return nullptr;
        }
      }
    }
    vxl_intrinsics_t di  = toCIntrinsics(*depth_intrin_cache_);
    vxl_intrinsics_t ci  = toCIntrinsics(*color_intrin_cache_);
    vxl_extrinsics_t ext = toCExtrinsics(*depth_to_color_ext_cache_);
    vxl_frame_t * out = nullptr;
    auto err = vxl_dip_align_depth_to_rgb(depth->handle(), &di, &ci, &ext, scale, &out);
    if (err != VXL_SUCCESS || !out) {return nullptr;}
    return vxl::Frame::create(out, /*own=*/ true);
  }

  static std::shared_ptr<vxl::FilterPipeline> buildFilterPipeline(const FilterChain & fc)
  {
    auto pipe = std::make_shared<vxl::FilterPipeline>();
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

  // ──────────────────────────────────────────────────────────────────────────
  // Members
  // ──────────────────────────────────────────────────────────────────────────

  // Serializes ALL direct SDK API calls (context_/device_/sensor/stream/
  // frame methods, vxl_dip_*) on this backend instance. vxlsdk is a
  // thread-unsafe device state machine; the stream callback path doesn't
  // reach into SDK from worker threads, but every caller-driven path
  // (open/setOption/publish-thread frame processing) does and must serialize.
  // recursive_mutex so internal helpers (setFilterChain → isOptionSupported
  // → setOption) don't self-deadlock.
  mutable std::recursive_mutex sdk_mutex_;

  vxl::ContextPtr context_;
  vxl::DevicePtr device_;

  // Owned per-physical-stream handles. nullptr when stream not enabled.
  // BOTH sensor and stream are kept alive: stream internally references
  // sensor state; dropping sensor while stream is alive crashes inside
  // backend stream_start (use-after-free on dev->lock acquisition).
  vxl::SensorPtr sensor_depth_;
  vxl::SensorPtr sensor_ir_;
  vxl::SensorPtr sensor_color_;
  vxl::StreamPtr stream_depth_;
  vxl::StreamPtr stream_ir_;
  vxl::StreamPtr stream_color_;

  // User callback + run state.
  BackendFrameSetCallback callback_;
  std::atomic<bool> running_{false};

  // Per-stream slot for latest frame received from SDK callback.
  StreamSlot slot_depth_;
  StreamSlot slot_ir_;
  StreamSlot slot_color_;

  // Which streams must contribute to a BackendFrameSet emission.
  std::atomic<bool> need_depth_{false};
  std::atomic<bool> need_ir_{false};
  std::atomic<bool> need_color_{false};
  std::string sync_mode_ = "approximate";

  // publish_thread + its wakeup cv.
  std::thread publish_thread_;
  std::mutex publish_mu_;
  std::condition_variable publish_cv_;

  // Host-side filter pipeline (rebuild on setFilterChain, snapshot in publish).
  std::mutex filter_mutex_;
  std::shared_ptr<vxl::FilterPipeline> filter_pipeline_;

  // Depth-to-color alignment.
  std::atomic<bool> align_enabled_{false};
  std::atomic<float> align_scale_{1.0f};
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
