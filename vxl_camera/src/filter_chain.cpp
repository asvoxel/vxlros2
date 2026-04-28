#include "vxl_camera/filter_chain.hpp"

#include <sstream>

namespace vxl_camera
{

std::string filterChainSummary(const FilterChain & fc)
{
  std::ostringstream os;
  bool any = false;
  auto append = [&](const std::string & label) {
      if (any) {os << ",";}
      os << label;
      any = true;
    };

  if (fc.decimation.enabled) {
    append("decim/" + std::to_string(fc.decimation.scale));
  }
  if (fc.threshold.enabled) {
    append("thresh/" + std::to_string(fc.threshold.min_mm) + "-" +
      std::to_string(fc.threshold.max_mm));
  }
  if (fc.spatial.enabled) {append("spatial");}
  if (fc.temporal.enabled) {append("temporal");}
  if (fc.hole_filling.enabled) {append("hole_fill");}

  return any ? os.str() : "off";
}

}  // namespace vxl_camera
