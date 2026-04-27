#include "vxl_camera/sensor_options.hpp"

namespace vxl_camera
{

const std::map<std::string, OptionBinding> & dynamicOptionTable()
{
  static const std::map<std::string, OptionBinding> table = {
    // Color sensor — exposure / gain / image
    {"color.exposure",           {vxl::SensorType::Color, VXL_OPTION_EXPOSURE}},
    {"color.gain",               {vxl::SensorType::Color, VXL_OPTION_GAIN}},
    {"color.auto_exposure",      {vxl::SensorType::Color, VXL_OPTION_AUTO_EXPOSURE}},
    {"color.brightness",         {vxl::SensorType::Color, VXL_OPTION_BRIGHTNESS}},
    {"color.contrast",           {vxl::SensorType::Color, VXL_OPTION_CONTRAST}},
    {"color.saturation",         {vxl::SensorType::Color, VXL_OPTION_SATURATION}},
    {"color.sharpness",          {vxl::SensorType::Color, VXL_OPTION_SHARPNESS}},
    {"color.white_balance",      {vxl::SensorType::Color, VXL_OPTION_WHITE_BALANCE}},
    {"color.auto_white_balance", {vxl::SensorType::Color, VXL_OPTION_AUTO_WHITE_BALANCE}},
    {"color.gamma",              {vxl::SensorType::Color, VXL_OPTION_GAMMA}},
    {"color.hue",                {vxl::SensorType::Color, VXL_OPTION_HUE}},
    // Depth sensor — exposure / range / IR
    {"depth.exposure",           {vxl::SensorType::Depth, VXL_OPTION_EXPOSURE}},
    {"depth.gain",               {vxl::SensorType::Depth, VXL_OPTION_GAIN}},
    {"depth.auto_exposure",      {vxl::SensorType::Depth, VXL_OPTION_AUTO_EXPOSURE}},
    {"depth.min_distance",       {vxl::SensorType::Depth, VXL_OPTION_MIN_DISTANCE}},
    {"depth.max_distance",       {vxl::SensorType::Depth, VXL_OPTION_MAX_DISTANCE}},
    {"depth.ir_enable",          {vxl::SensorType::Depth, VXL_OPTION_IR_ENABLE}},
  };
  return table;
}

const std::set<std::string> & coldParameters()
{
  static const std::set<std::string> cold = {
    "device_serial", "output_mode",
    "color.width", "color.height", "color.fps",
    "depth.width", "depth.height", "depth.fps",
    "ir.width", "ir.height", "ir.fps",
    "sync_mode", "frame_queue_size",
    "tf_prefix", "publish_tf",
    "point_cloud.enabled",
  };
  return cold;
}

}  // namespace vxl_camera
