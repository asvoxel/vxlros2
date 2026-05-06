#ifndef VXL_CAMERA__SENSOR_OPTIONS_HPP_
#define VXL_CAMERA__SENSOR_OPTIONS_HPP_

#include <rclcpp/parameter.hpp>
#include <vxl.hpp>

#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace vxl_camera
{

// Maps a ROS2 parameter name to a (sensor, vxl_option) pair for runtime updates.
// Used by both VxlCameraNode and VxlCameraLifecycleNode.
struct OptionBinding {
  vxl::SensorType sensor;
  vxl_option_t option;
};

const std::map<std::string, OptionBinding> & dynamicOptionTable();

// Parameters that are declared at startup and CANNOT be changed at runtime.
// Setting one of these via `ros2 param set` will be rejected with a clear reason.
const std::set<std::string> & coldParameters();

// Parse a sensor selector string ("color" | "depth" | "ir") to vxl::SensorType.
// Replaces the prior heuristic that routed by option id range (>=100 → depth)
// which silently misrouted any new option whose id happened to overlap.
std::optional<vxl::SensorType> parseSensorSelector(std::string_view s);

// Result of an option dependency check on a proposed parameter batch.
// `ok == false` means the batch must be rejected; `reason` is the message
// to surface back to the user via SetParametersResult.
struct OptionDependencyResult {
  bool ok = true;
  std::string reason;
};

// Reject manual exposure/gain/white_balance changes when the corresponding
// auto-mode is (or will be after this batch) enabled. Examples:
//   auto_exposure = 1  AND  exposure = 5000   → rejected
//   auto_exposure = 0  AND  exposure = 5000   → accepted (auto turned off)
//
// `params` is the proposed batch from the on_set_parameters_callback.
// `current_auto_*` are the currently-committed values of the auto-mode
// integer params (caller passes 0 if the param doesn't exist on the device).
OptionDependencyResult checkOptionDependencies(
  const std::vector<rclcpp::Parameter> & params,
  int current_color_auto_exposure,
  int current_color_auto_wb,
  int current_depth_auto_exposure);

}  // namespace vxl_camera

#endif  // VXL_CAMERA__SENSOR_OPTIONS_HPP_
