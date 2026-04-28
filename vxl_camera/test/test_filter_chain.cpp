// Tests for FilterChain config + parameter override merge logic.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "vxl_camera/filter_chain.hpp"
#include "vxl_camera/mock_camera_backend.hpp"

using vxl_camera::FilterChain;
using vxl_camera::filterChainSummary;
using vxl_camera::readFilterChainParams;
using vxl_camera::applyFilterParamOverrides;

class FilterChainTest : public ::testing::Test
{
protected:
  void SetUp() override {if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}}
  void TearDown() override {if (rclcpp::ok()) {rclcpp::shutdown();}}
};

TEST_F(FilterChainTest, DefaultIsAllDisabled)
{
  FilterChain fc;
  EXPECT_FALSE(fc.decimation.enabled);
  EXPECT_FALSE(fc.threshold.enabled);
  EXPECT_FALSE(fc.spatial.enabled);
  EXPECT_FALSE(fc.temporal.enabled);
  EXPECT_FALSE(fc.hole_filling.enabled);
  EXPECT_EQ(filterChainSummary(fc), "off");
}

TEST_F(FilterChainTest, SummaryStringListsEnabledFilters)
{
  FilterChain fc;
  fc.hole_filling.enabled = true;
  fc.spatial.enabled = true;
  EXPECT_EQ(filterChainSummary(fc), "spatial,hole_fill");
}

TEST_F(FilterChainTest, ReadFromNodeWithoutDeclaredParamsUsesDefaults)
{
  auto node = std::make_shared<rclcpp::Node>("test_filter_undeclared");
  auto fc = readFilterChainParams(*node);
  // None declared → all defaults
  EXPECT_FALSE(fc.decimation.enabled);
  EXPECT_EQ(fc.decimation.scale, 2);
  EXPECT_EQ(fc.threshold.min_mm, 100u);
  EXPECT_EQ(fc.threshold.max_mm, 5000u);
  EXPECT_FLOAT_EQ(fc.spatial.alpha, 0.5f);
  EXPECT_FALSE(fc.hole_filling.enabled);
}

TEST_F(FilterChainTest, ReadFromNodeReturnsDeclaredValues)
{
  auto node = std::make_shared<rclcpp::Node>("test_filter_declared");
  node->declare_parameter("filters.hole_filling.enabled", true);
  node->declare_parameter("filters.hole_filling.mode", 1);
  node->declare_parameter("filters.spatial.enabled", true);
  node->declare_parameter("filters.spatial.alpha", 0.7);

  auto fc = readFilterChainParams(*node);
  EXPECT_TRUE(fc.hole_filling.enabled);
  EXPECT_EQ(fc.hole_filling.mode, 1);
  EXPECT_TRUE(fc.spatial.enabled);
  EXPECT_FLOAT_EQ(fc.spatial.alpha, 0.7f);
}

TEST_F(FilterChainTest, ApplyOverridesRespectsCallbackParams)
{
  // Simulates the on_set_parameters callback: we have the proposed new values
  // in `params` and a starting FilterChain; the merge should pick up the new
  // values for matching names.
  FilterChain fc;
  fc.spatial.alpha = 0.5f;
  fc.hole_filling.enabled = false;

  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("filters.hole_filling.enabled", true),
    rclcpp::Parameter("filters.spatial.alpha", 0.9),
    rclcpp::Parameter("color.exposure", 7000),  // unrelated — ignored
  };
  applyFilterParamOverrides(fc, params);
  EXPECT_TRUE(fc.hole_filling.enabled);
  EXPECT_FLOAT_EQ(fc.spatial.alpha, 0.9f);
}

TEST_F(FilterChainTest, ApplyOverridesIgnoresNonFilterParams)
{
  FilterChain fc_before;
  FilterChain fc = fc_before;
  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("color.gain", 100),
    rclcpp::Parameter("output_mode", "rgbd"),
  };
  applyFilterParamOverrides(fc, params);
  // No filter params changed → fc unchanged
  EXPECT_EQ(fc.spatial.enabled, fc_before.spatial.enabled);
  EXPECT_EQ(fc.hole_filling.enabled, fc_before.hole_filling.enabled);
}

TEST_F(FilterChainTest, DeviceFiltersDefaultOff)
{
  FilterChain fc;
  EXPECT_FALSE(fc.device.denoise.enabled);
  EXPECT_EQ(fc.device.denoise.level, 2);
  EXPECT_FALSE(fc.device.median.enabled);
  EXPECT_EQ(fc.device.median.kernel_size, 3);
  EXPECT_FALSE(fc.device.outlier_removal.enabled);
}

TEST_F(FilterChainTest, SummaryIncludesDeviceFilters)
{
  FilterChain fc;
  fc.device.denoise.enabled = true;
  fc.device.denoise.level = 3;
  fc.device.outlier_removal.enabled = true;
  EXPECT_EQ(filterChainSummary(fc), "dev_denoise/3,dev_outlier");
}

TEST_F(FilterChainTest, ApplyOverridesPicksUpDeviceFilters)
{
  FilterChain fc;
  std::vector<rclcpp::Parameter> params{
    rclcpp::Parameter("filters.device.denoise.enabled", true),
    rclcpp::Parameter("filters.device.denoise.level", 3),
    rclcpp::Parameter("filters.device.median.enabled", true),
    rclcpp::Parameter("filters.device.median.kernel_size", 5),
    rclcpp::Parameter("filters.device.outlier_removal.enabled", true),
  };
  applyFilterParamOverrides(fc, params);
  EXPECT_TRUE(fc.device.denoise.enabled);
  EXPECT_EQ(fc.device.denoise.level, 3);
  EXPECT_TRUE(fc.device.median.enabled);
  EXPECT_EQ(fc.device.median.kernel_size, 5);
  EXPECT_TRUE(fc.device.outlier_removal.enabled);
}

TEST_F(FilterChainTest, MockBackendRecordsSetFilterChain)
{
  auto mock = std::make_shared<vxl_camera::MockCameraBackend>();
  EXPECT_EQ(mock->filterChainSetCount(), 0);

  FilterChain fc;
  fc.hole_filling.enabled = true;
  fc.hole_filling.mode = 2;
  mock->setFilterChain(fc);

  EXPECT_EQ(mock->filterChainSetCount(), 1);
  auto recorded = mock->lastFilterChain();
  EXPECT_TRUE(recorded.hole_filling.enabled);
  EXPECT_EQ(recorded.hole_filling.mode, 2);
}
