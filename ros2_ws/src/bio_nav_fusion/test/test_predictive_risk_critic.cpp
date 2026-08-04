#include <string>

#include "bio_nav_fusion/predictive_risk_critic.hpp"
#include "gtest/gtest.h"

namespace
{
bio_nav_interfaces::msg::PredictiveRiskGrid healthyGrid()
{
  bio_nav_interfaces::msg::PredictiveRiskGrid grid;
  grid.header.frame_id = "base_link";
  grid.reset_epoch = 3U;
  grid.map_version = "map-v1";
  grid.model_sha256 = std::string(64U, 'a');
  grid.calibration_sha256 = std::string(64U, 'b');
  grid.qualification_sha256 = std::string(64U, 'c');
  grid.width = 32U;
  grid.height = 32U;
  grid.resolution_m = 0.5F;
  grid.origin_x = -8.0F;
  grid.origin_y = -8.0F;
  grid.horizons_s = {0.2F, 0.4F, 0.8F};
  grid.healthy = true;
  grid.reliability = 0.9F;
  grid.ood_probability = 0.1F;
  grid.rejection_mask = 0U;
  grid.visibility.fill(1U);
  grid.risk.fill(0.0F);
  return grid;
}
}  // namespace

TEST(PredictiveRiskCritic, accepts_only_complete_frozen_identity)
{
  const auto grid = healthyGrid();
  EXPECT_EQ(
    mppi::critics::PredictiveRiskCritic::validateRisk(
      &grid, 0.1, 0.3, 0.6, 0.4, 3U, "base_link", "map-v1",
      std::string(64U, 'a'), std::string(64U, 'b'), std::string(64U, 'c')),
    "");
  EXPECT_EQ(
    mppi::critics::PredictiveRiskCritic::validateRisk(
      &grid, 0.1, 0.3, 0.6, 0.4, 3U, "base_link", "map-v1",
      std::string(64U, '0'), std::string(64U, 'b'), std::string(64U, 'c')),
    "identity_unconfigured");
  EXPECT_EQ(
    mppi::critics::PredictiveRiskCritic::validateRisk(
      &grid, 0.4, 0.3, 0.6, 0.4, 3U, "base_link", "map-v1",
      std::string(64U, 'a'), std::string(64U, 'b'), std::string(64U, 'c')),
    "stale");
}

TEST(PredictiveRiskCritic, samples_three_horizons_without_extrapolating_grid)
{
  auto grid = healthyGrid();
  const std::size_t cell = 16U * 32U + 16U;
  grid.risk[cell] = 0.2F;
  grid.risk[1024U + cell] = 0.4F;
  grid.risk[2048U + cell] = 0.8F;
  EXPECT_FLOAT_EQ(
    mppi::critics::PredictiveRiskCritic::sampleRisk(grid, 0.1F, 0.1F, 0.2F),
    0.2F);
  EXPECT_FLOAT_EQ(
    mppi::critics::PredictiveRiskCritic::sampleRisk(grid, 0.1F, 0.1F, 0.4F),
    0.4F);
  EXPECT_FLOAT_EQ(
    mppi::critics::PredictiveRiskCritic::sampleRisk(grid, 0.1F, 0.1F, 0.8F),
    0.8F);
  EXPECT_FLOAT_EQ(
    mppi::critics::PredictiveRiskCritic::sampleRisk(grid, 20.0F, 20.0F, 0.8F),
    0.0F);
}
