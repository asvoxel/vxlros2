#include "vxl_camera/stream_manager.hpp"

namespace vxl_camera
{

StreamManager::StreamManager(vxl::DevicePtr device, const StreamParams & params)
: device_(device), pipeline_(device), params_(params)
{
  // Set sync mode
  if (params.sync_mode == "strict") {
    pipeline_.setSyncMode(vxl::SyncMode::Strict);
  } else if (params.sync_mode == "approximate") {
    pipeline_.setSyncMode(vxl::SyncMode::Approximate);
  } else {
    pipeline_.setSyncMode(vxl::SyncMode::None);
  }

  pipeline_.setFrameQueueSize(params.frame_queue_size);
}

StreamManager::~StreamManager()
{
  stop();
}

void StreamManager::enableColor()
{
  pipeline_.enableStream(
    vxl::SensorType::Color, vxl::Format::BGR,
    params_.color_width, params_.color_height, params_.color_fps);
}

void StreamManager::enableDepth()
{
  pipeline_.enableStream(
    vxl::SensorType::Depth, vxl::Format::Z16,
    params_.depth_width, params_.depth_height, params_.depth_fps);
}

void StreamManager::enableIR()
{
  pipeline_.enableStream(
    vxl::SensorType::IR, vxl::Format::Gray16,
    params_.ir_width, params_.ir_height, params_.ir_fps);
}

void StreamManager::start(FrameSetCallback callback)
{
  if (running_) {
    return;
  }

  // 1. Store callback before anything starts (no race possible)
  callback_ = std::move(callback);

  // 2. Start pipeline first (may throw if profile not found etc.)
  pipeline_.start();

  // 3. Only after pipeline is running, start poll thread
  running_ = true;
  try {
    poll_thread_ = std::thread(&StreamManager::pollThread, this);
  } catch (...) {
    // Thread creation failed — clean up pipeline
    running_ = false;
    pipeline_.stop();
    throw;
  }
}

void StreamManager::stop()
{
  // 1. Signal thread to exit
  running_ = false;

  // 2. Join thread first (it calls pipeline_.waitForFrameSet which will
  //    return quickly once pipeline is stopped or via timeout)
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }

  // 3. Stop pipeline after thread is done
  if (pipeline_.isRunning()) {
    pipeline_.stop();
  }
}

bool StreamManager::isRunning() const
{
  return running_;
}

void StreamManager::pollThread()
{
  while (running_) {
    try {
      auto frameset = pipeline_.waitForFrameSet(100);
      if (frameset && callback_) {
        callback_(frameset);
      }
    } catch (const vxl::Error &) {
      // Timeout or transient error, continue polling
    }
  }
}

}  // namespace vxl_camera
