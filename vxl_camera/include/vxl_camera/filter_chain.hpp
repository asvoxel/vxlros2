#ifndef VXL_CAMERA__FILTER_CHAIN_HPP_
#define VXL_CAMERA__FILTER_CHAIN_HPP_

#include <cstdint>
#include <string>
#include <vector>

// Depth post-processing filter chain configuration.
//
// Mirrors the host-side filters provided by vxl::FilterPipeline (vxl_filter.hpp):
// Decimation → Threshold → Spatial → Temporal → HoleFilling. Each filter is
// independently togglable and tunable from ROS2 parameters.
//
// V1 scope (this commit): host-side filters only (always available, all SKUs).
// V2 (future): auto/device modes that delegate to VXL6X5 device-side filters
// (DENOISE/MEDIAN/OUTLIER) when the connected device supports them and falls
// back to host-side otherwise.
namespace vxl_camera
{

struct FilterChain {
  // Decimation: subsample depth by integer factor (2/4/8). Reduces noise + CPU.
  struct Decimation {
    bool enabled = false;
    int scale = 2;
  } decimation;

  // Threshold: clip depth values outside [min_mm, max_mm] to 0 (invalid).
  struct Threshold {
    bool enabled = false;
    uint16_t min_mm = 100;
    uint16_t max_mm = 5000;
  } threshold;

  // Spatial: edge-preserving spatial smoothing (reduces noise, preserves boundaries).
  struct Spatial {
    bool enabled = false;
    int magnitude = 2;       // 1..5
    float alpha = 0.5f;      // 0.25..1.0
    float delta = 20.0f;     // 1..50
  } spatial;

  // Temporal: multi-frame temporal averaging (reduces flicker).
  struct Temporal {
    bool enabled = false;
    float alpha = 0.4f;      // 0.0..1.0 (lower = smoother, more lag)
    float delta = 20.0f;     // 1..100
  } temporal;

  // Hole filling: fill invalid (zero) depth pixels from neighbors.
  // mode: 0 = NearestFromAround (default, safest)
  //       1 = FarestFromAround
  //       2 = FillFromLeft (fastest, may bleed)
  struct HoleFilling {
    bool enabled = false;
    int mode = 0;
  } hole_filling;

  // ── Device-side filters (VXL6X5 only) ─────────────────────────────────
  // These run on the camera FPGA — zero host CPU. Silently no-op on devices
  // that don't expose the corresponding options (e.g. VXL435), so it's safe
  // to leave them on in shared launch configs.
  struct Device {
    // Denoise (VXL6X5_OPTION_DENOISE_ENABLE / DENOISE_LEVEL).
    struct Denoise {
      bool enabled = false;
      int level = 2;          // 1..3
    } denoise;
    // Median filter on depth (VXL6X5_OPTION_MEDIAN_FILTER / MEDIAN_KERNEL_SIZE).
    struct Median {
      bool enabled = false;
      int kernel_size = 3;    // odd, typically 3 or 5
    } median;
    // Outlier removal (VXL6X5_OPTION_OUTLIER_REMOVAL).
    struct Outlier {
      bool enabled = false;
    } outlier_removal;
  } device;
};

// Build a string summary of which filters are enabled — useful for diagnostics.
std::string filterChainSummary(const FilterChain & fc);

// Standard parameter names (under "filters." prefix). Listed here so the
// node-level code and onParameterChange detection stay in sync.
constexpr const char * kFilterParamPrefix = "filters.";

// Apply pending parameter overrides (from an OnSetParametersCallback) to a
// FilterChain. The callback fires BEFORE rclcpp commits values, so reading
// via get_parameter() inside the callback yields stale values; the new ones
// must come from the `params` argument instead.
template<typename ParamT>
void applyFilterParamOverrides(FilterChain & fc, const std::vector<ParamT> & params)
{
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n.rfind(kFilterParamPrefix, 0) != 0) {continue;}
    if (n == "filters.decimation.enabled") {fc.decimation.enabled = p.as_bool();}
    else if (n == "filters.decimation.scale") {fc.decimation.scale = static_cast<int>(p.as_int());}
    else if (n == "filters.threshold.enabled") {fc.threshold.enabled = p.as_bool();}
    else if (n == "filters.threshold.min_mm") {
      fc.threshold.min_mm = static_cast<uint16_t>(p.as_int());
    }
    else if (n == "filters.threshold.max_mm") {
      fc.threshold.max_mm = static_cast<uint16_t>(p.as_int());
    }
    else if (n == "filters.spatial.enabled") {fc.spatial.enabled = p.as_bool();}
    else if (n == "filters.spatial.magnitude") {fc.spatial.magnitude = static_cast<int>(p.as_int());}
    else if (n == "filters.spatial.alpha") {fc.spatial.alpha = static_cast<float>(p.as_double());}
    else if (n == "filters.spatial.delta") {fc.spatial.delta = static_cast<float>(p.as_double());}
    else if (n == "filters.temporal.enabled") {fc.temporal.enabled = p.as_bool();}
    else if (n == "filters.temporal.alpha") {fc.temporal.alpha = static_cast<float>(p.as_double());}
    else if (n == "filters.temporal.delta") {fc.temporal.delta = static_cast<float>(p.as_double());}
    else if (n == "filters.hole_filling.enabled") {fc.hole_filling.enabled = p.as_bool();}
    else if (n == "filters.hole_filling.mode") {
      fc.hole_filling.mode = static_cast<int>(p.as_int());
    }
    else if (n == "filters.device.denoise.enabled") {fc.device.denoise.enabled = p.as_bool();}
    else if (n == "filters.device.denoise.level") {
      fc.device.denoise.level = static_cast<int>(p.as_int());
    }
    else if (n == "filters.device.median.enabled") {fc.device.median.enabled = p.as_bool();}
    else if (n == "filters.device.median.kernel_size") {
      fc.device.median.kernel_size = static_cast<int>(p.as_int());
    }
    else if (n == "filters.device.outlier_removal.enabled") {
      fc.device.outlier_removal.enabled = p.as_bool();
    }
  }
}

// Helper: build a FilterChain from a node by reading the standard parameters.
// Falls back to defaults for any parameter that hasn't been declared (so tests
// that don't declare all of them still work).
template<typename NodeT>
FilterChain readFilterChainParams(NodeT & node)
{
  FilterChain fc;
  auto get_bool = [&node](const std::string & n, bool d) {
      return node.has_parameter(n) ? node.get_parameter(n).as_bool() : d;
    };
  auto get_int = [&node](const std::string & n, int d) {
      return node.has_parameter(n) ?
             static_cast<int>(node.get_parameter(n).as_int()) : d;
    };
  auto get_dbl = [&node](const std::string & n, double d) {
      return node.has_parameter(n) ?
             static_cast<float>(node.get_parameter(n).as_double()) :
             static_cast<float>(d);
    };

  fc.decimation.enabled = get_bool("filters.decimation.enabled", false);
  fc.decimation.scale = get_int("filters.decimation.scale", 2);

  fc.threshold.enabled = get_bool("filters.threshold.enabled", false);
  fc.threshold.min_mm = static_cast<uint16_t>(get_int("filters.threshold.min_mm", 100));
  fc.threshold.max_mm = static_cast<uint16_t>(get_int("filters.threshold.max_mm", 5000));

  fc.spatial.enabled = get_bool("filters.spatial.enabled", false);
  fc.spatial.magnitude = get_int("filters.spatial.magnitude", 2);
  fc.spatial.alpha = get_dbl("filters.spatial.alpha", 0.5);
  fc.spatial.delta = get_dbl("filters.spatial.delta", 20.0);

  fc.temporal.enabled = get_bool("filters.temporal.enabled", false);
  fc.temporal.alpha = get_dbl("filters.temporal.alpha", 0.4);
  fc.temporal.delta = get_dbl("filters.temporal.delta", 20.0);

  fc.hole_filling.enabled = get_bool("filters.hole_filling.enabled", false);
  fc.hole_filling.mode = get_int("filters.hole_filling.mode", 0);

  // Device-side
  fc.device.denoise.enabled = get_bool("filters.device.denoise.enabled", false);
  fc.device.denoise.level = get_int("filters.device.denoise.level", 2);
  fc.device.median.enabled = get_bool("filters.device.median.enabled", false);
  fc.device.median.kernel_size = get_int("filters.device.median.kernel_size", 3);
  fc.device.outlier_removal.enabled =
    get_bool("filters.device.outlier_removal.enabled", false);
  return fc;
}

}  // namespace vxl_camera

#endif  // VXL_CAMERA__FILTER_CHAIN_HPP_
