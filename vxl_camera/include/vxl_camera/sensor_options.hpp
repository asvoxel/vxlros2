#ifndef VXL_CAMERA__SENSOR_OPTIONS_HPP_
#define VXL_CAMERA__SENSOR_OPTIONS_HPP_

#include <vxl.hpp>

#include <map>
#include <set>
#include <string>

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

}  // namespace vxl_camera

#endif  // VXL_CAMERA__SENSOR_OPTIONS_HPP_
