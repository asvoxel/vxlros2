#include <gtest/gtest.h>

#include "vxl_camera/sensor_options.hpp"

#include <set>

using vxl_camera::dynamicOptionTable;
using vxl_camera::coldParameters;

TEST(SensorOptions, DynamicTableHasExpectedSize)
{
  // 11 color options + 6 depth options = 17 total.
  // Update this number AND the table when adding new bindings.
  EXPECT_EQ(dynamicOptionTable().size(), 17u);
}

TEST(SensorOptions, AllRequiredHotParamsPresent)
{
  // Spot-check critical bindings users rely on (exposure/gain on both sensors).
  const auto & t = dynamicOptionTable();
  EXPECT_NE(t.find("color.exposure"),     t.end());
  EXPECT_NE(t.find("color.gain"),         t.end());
  EXPECT_NE(t.find("color.auto_exposure"), t.end());
  EXPECT_NE(t.find("depth.exposure"),     t.end());
  EXPECT_NE(t.find("depth.gain"),         t.end());
  EXPECT_NE(t.find("depth.min_distance"), t.end());
  EXPECT_NE(t.find("depth.max_distance"), t.end());
  EXPECT_NE(t.find("depth.ir_enable"),    t.end());
}

TEST(SensorOptions, ColorOptionsRouteToColorSensor)
{
  for (const auto & [name, binding] : dynamicOptionTable()) {
    if (name.rfind("color.", 0) == 0) {
      EXPECT_EQ(binding.sensor, vxl::SensorType::Color)
        << "Param '" << name << "' should target Color sensor";
    }
  }
}

TEST(SensorOptions, DepthOptionsRouteToDepthSensor)
{
  for (const auto & [name, binding] : dynamicOptionTable()) {
    if (name.rfind("depth.", 0) == 0) {
      EXPECT_EQ(binding.sensor, vxl::SensorType::Depth)
        << "Param '" << name << "' should target Depth sensor";
    }
  }
}

TEST(SensorOptions, NoColdHotOverlap)
{
  // A parameter must not be both "cold" (rejected at runtime) AND "hot" (routed to SDK).
  const auto & cold = coldParameters();
  for (const auto & [name, _] : dynamicOptionTable()) {
    EXPECT_EQ(cold.count(name), 0u)
      << "Parameter '" << name << "' is both hot and cold — fix sensor_options.cpp";
  }
}

TEST(SensorOptions, ColdSetCoversBaselineParams)
{
  // These cold parameters drive stream config — runtime change requires restart.
  const auto & cold = coldParameters();
  for (const auto & name : {
      "device_serial", "output_mode",
      "color.width", "color.height", "color.fps",
      "depth.width", "depth.height", "depth.fps",
      "ir.width", "ir.height", "ir.fps",
      "sync_mode", "frame_queue_size",
      "tf_prefix", "publish_tf",
      "point_cloud.enabled"})
  {
    EXPECT_EQ(cold.count(name), 1u) << "Missing cold param: " << name;
  }
}

TEST(SensorOptions, DistinctSdkOptionsPerSensor)
{
  // Within one sensor, no two parameter names should map to the same vxl_option.
  std::set<vxl_option_t> color_seen;
  std::set<vxl_option_t> depth_seen;
  for (const auto & [name, b] : dynamicOptionTable()) {
    auto & target = (b.sensor == vxl::SensorType::Color) ? color_seen : depth_seen;
    EXPECT_TRUE(target.insert(b.option).second)
      << "Duplicate SDK option for '" << name << "'";
  }
}

TEST(SensorOptions, TableIsStableAcrossCalls)
{
  // The table is a function-local static; verify the same reference is returned.
  EXPECT_EQ(&dynamicOptionTable(), &dynamicOptionTable());
  EXPECT_EQ(&coldParameters(), &coldParameters());
}
