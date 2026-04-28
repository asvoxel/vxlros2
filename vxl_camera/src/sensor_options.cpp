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

OptionDependencyResult checkOptionDependencies(
  const std::vector<rclcpp::Parameter> & params,
  int current_color_auto_exposure,
  int current_color_auto_wb,
  int current_depth_auto_exposure)
{
  // Compute the EFFECTIVE auto-mode state for this batch: start from current
  // committed values, overlay proposals from `params`. This way a batch like
  // {color.auto_exposure=0, color.exposure=5000} is accepted (auto is being
  // turned off in the same transaction).
  int eff_color_ae = current_color_auto_exposure;
  int eff_color_awb = current_color_auto_wb;
  int eff_depth_ae = current_depth_auto_exposure;
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "color.auto_exposure") {eff_color_ae = static_cast<int>(p.as_int());}
    else if (n == "color.auto_white_balance") {eff_color_awb = static_cast<int>(p.as_int());}
    else if (n == "depth.auto_exposure") {eff_depth_ae = static_cast<int>(p.as_int());}
  }

  for (const auto & p : params) {
    const auto & n = p.get_name();
    if ((n == "color.exposure" || n == "color.gain") && eff_color_ae > 0) {
      return {false,
        "'" + n + "' cannot be set while 'color.auto_exposure' is enabled. "
        "Set color.auto_exposure=0 first (or in the same batch)."};
    }
    if (n == "color.white_balance" && eff_color_awb > 0) {
      return {false,
        "'color.white_balance' cannot be set while 'color.auto_white_balance' "
        "is enabled. Disable auto WB first (or in the same batch)."};
    }
    if ((n == "depth.exposure" || n == "depth.gain") && eff_depth_ae > 0) {
      return {false,
        "'" + n + "' cannot be set while 'depth.auto_exposure' is enabled. "
        "Set depth.auto_exposure=0 first (or in the same batch)."};
    }
  }
  return {true, ""};
}

}  // namespace vxl_camera
