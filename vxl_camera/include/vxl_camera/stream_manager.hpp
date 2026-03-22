#ifndef VXL_CAMERA__STREAM_MANAGER_HPP_
#define VXL_CAMERA__STREAM_MANAGER_HPP_

#include <vxl.hpp>
#include <functional>
#include <atomic>
#include <thread>

namespace vxl_camera
{

struct StreamParams {
  uint32_t color_width = 1280;
  uint32_t color_height = 720;
  uint32_t color_fps = 30;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps = 30;
  uint32_t ir_width = 640;
  uint32_t ir_height = 480;
  uint32_t ir_fps = 30;
  std::string sync_mode = "strict";
  size_t frame_queue_size = 4;
};

class StreamManager
{
public:
  using FrameSetCallback = std::function<void(vxl::FrameSetPtr)>;

  StreamManager(vxl::DevicePtr device, const StreamParams & params);
  ~StreamManager();

  void enableColor();
  void enableDepth();
  void enableIR();

  void start(FrameSetCallback callback);
  void stop();
  bool isRunning() const;

  vxl::DevicePtr device() const { return device_; }

private:
  void pollThread();

  vxl::DevicePtr device_;
  vxl::Pipeline pipeline_;
  StreamParams params_;
  FrameSetCallback callback_;
  std::thread poll_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace vxl_camera

#endif  // VXL_CAMERA__STREAM_MANAGER_HPP_
