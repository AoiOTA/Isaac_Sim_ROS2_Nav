#include <array>
#include <limits>

#include "bio_nav_fusion/bio_nav_grid_based.hpp"
#include "bio_nav_fusion/cognitive_risk_layer.hpp"
#include "bio_nav_fusion/reachability_observer_layer.hpp"
#include "bio_nav_fusion/local_risk_grid_layer.hpp"
#include "gtest/gtest.h"
#include "nav2_costmap_2d/cost_values.hpp"

TEST(BioNavGridBased, higher_prior_breaks_equal_primary_cost_toward_top_route)
{
  nav2_costmap_2d::Costmap2D costmap(5, 3, 1.0, -2.5, -1.5, 0);
  costmap.setCost(2, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  start.header.frame_id = "map";
  goal.header.frame_id = "map";
  start.pose.position.x = -2.0;
  start.pose.position.y = 0.0;
  goal.pose.position.x = 2.0;
  goal.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal.pose.orientation.w = 1.0;
  std::array<float, 256> score{};
  // Costmap top row has world y=1 and therefore cognitive canvas row 9.
  for (int column = 5; column <= 10; ++column) {
    score[9 * 16 + column] = 1.0F;
  }
  const auto result = bio_nav_fusion::BioNavGridBased::equalCostSearch(
    costmap, start, goal, score, true, "map", rclcpp::Time(1, 0),
    []() {return false;});
  ASSERT_TRUE(result.success) << result.error;
  ASSERT_GT(result.path.poses.size(), 2u);
  bool used_top = false;
  for (const auto & pose : result.path.poses) {
    used_top = used_top || pose.pose.position.y > 0.5;
    EXPECT_GE(pose.pose.position.y, -0.5);
  }
  EXPECT_TRUE(used_top);
  EXPECT_GT(result.expanded_nodes, 0u);
}

TEST(BioNavGridBased, blocked_goal_fails_for_stock_fallback)
{
  nav2_costmap_2d::Costmap2D costmap(3, 3, 1.0, -1.5, -1.5, 0);
  costmap.setCost(2, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::PoseStamped start;
  geometry_msgs::msg::PoseStamped goal;
  start.pose.position.x = -1.0;
  start.pose.position.y = 0.0;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 0.0;
  std::array<float, 256> score{};
  const auto result = bio_nav_fusion::BioNavGridBased::equalCostSearch(
    costmap, start, goal, score, true, "map", rclcpp::Time(1, 0),
    []() {return false;});
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.error, "start_or_goal_blocked");
}

TEST(BioNavGridBased, module2_identity_must_be_recent_for_cognitive_planning)
{
  using bio_nav_fusion::BioNavGridBased;
  EXPECT_TRUE(BioNavGridBased::priorIdentityFresh(0.0, 0.5));
  EXPECT_TRUE(BioNavGridBased::priorIdentityFresh(0.5, 0.5));
  EXPECT_FALSE(BioNavGridBased::priorIdentityFresh(0.5001, 0.5));
  EXPECT_FALSE(BioNavGridBased::priorIdentityFresh(-0.1, 0.5));
  EXPECT_FALSE(BioNavGridBased::priorIdentityFresh(
      std::numeric_limits<double>::quiet_NaN(), 0.5));
}

TEST(ReachabilityObserverLayer, occupancy_snapshot_is_conservative)
{
  using bio_nav_fusion::ReachabilityObserverLayer;
  EXPECT_EQ(ReachabilityObserverLayer::occupancyValue(0), 0);
  EXPECT_EQ(ReachabilityObserverLayer::occupancyValue(126), 50);
  EXPECT_EQ(ReachabilityObserverLayer::occupancyValue(252), 100);
  EXPECT_EQ(ReachabilityObserverLayer::occupancyValue(253), 100);
  EXPECT_EQ(ReachabilityObserverLayer::occupancyValue(254), 100);
  EXPECT_EQ(ReachabilityObserverLayer::occupancyValue(255), -1);
}

TEST(CognitiveRiskLayer, calibrated_cost_is_thresholded_nonlethal_and_decays)
{
  using bio_nav_fusion::CognitiveRiskLayer;
  EXPECT_EQ(CognitiveRiskLayer::mapRiskCost(0.49F, 0.5F, 1.0, 80), 0);
  EXPECT_EQ(CognitiveRiskLayer::mapRiskCost(0.5F, 0.5F, 1.0, 80), 1);
  EXPECT_EQ(CognitiveRiskLayer::mapRiskCost(1.0F, 0.5F, 1.0, 80), 80);
  const auto decayed =
    CognitiveRiskLayer::mapRiskCost(1.0F, 0.5F, 0.5, 80);
  EXPECT_GT(decayed, 1);
  EXPECT_LT(decayed, 80);
  EXPECT_EQ(CognitiveRiskLayer::mapRiskCost(1.0F, 0.5F, 0.0, 80), 0);
}

TEST(CognitiveRiskLayer, active_risk_requires_a_healthy_threshold_crossing)
{
  using bio_nav_fusion::CognitiveRiskLayer;
  bio_nav_interfaces::msg::PlanningPrior prior;
  prior.risk_healthy = true;
  prior.risk_threshold = 0.5F;
  prior.dynamic_cost.fill(0.0F);
  EXPECT_FALSE(CognitiveRiskLayer::containsActiveRisk(prior));
  prior.dynamic_cost[42] = 0.5F;
  EXPECT_TRUE(CognitiveRiskLayer::containsActiveRisk(prior));
  prior.risk_rejection_mask = 4;
  EXPECT_FALSE(CognitiveRiskLayer::containsActiveRisk(prior));
  prior.risk_rejection_mask = 0;
  prior.risk_healthy = false;
  EXPECT_FALSE(CognitiveRiskLayer::containsActiveRisk(prior));
  prior.risk_healthy = true;
  prior.dynamic_cost[42] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(CognitiveRiskLayer::containsActiveRisk(prior));
}

TEST(CognitiveRiskLayer, fault_matrix_rejects_untrusted_risk_inputs)
{
  using bio_nav_fusion::CognitiveRiskLayer;
  bio_nav_interfaces::msg::PlanningPrior prior;
  prior.schema_version = "bio_nav_planning_prior_v4";
  prior.risk_healthy = true;
  prior.risk_reliability = 0.9F;
  prior.map_version = "map";
  prior.reset_epoch = 3;
  prior.risk_model_sha256 = "model";
  prior.qualification_receipt_sha256 = "qualification";
  prior.risk_threshold = 0.5F;
  prior.risk_ttl_s = 0.8F;
  prior.dynamic_cost.fill(0.0F);
  const auto validate = [&prior]() {
    return CognitiveRiskLayer::validatePrior(
      &prior, 0.1, 0.5, 0.2, 3, "map", "model", "qualification");
  };

  EXPECT_EQ(validate(), "");
  EXPECT_EQ(
    CognitiveRiskLayer::validatePrior(
      &prior, 0.6, 0.5, 0.2, 3, "map", "model", "qualification"),
    "stale");

  prior.risk_healthy = false;
  EXPECT_EQ(validate(), "risk_unhealthy");
  prior.risk_healthy = true;
  prior.risk_reliability = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(validate(), "risk_unhealthy");
  prior.risk_reliability = 0.9F;

  prior.risk_rejection_mask = 4;
  EXPECT_EQ(validate(), "risk_rejected");
  prior.risk_rejection_mask = 0;

  prior.map_version = "old-map";
  EXPECT_EQ(validate(), "map_reset_mismatch");
  prior.map_version = "map";
  prior.reset_epoch = 2;
  EXPECT_EQ(validate(), "map_reset_mismatch");
  prior.reset_epoch = 3;

  prior.risk_model_sha256 = "wrong-model";
  EXPECT_EQ(validate(), "model_hash_mismatch");
  prior.risk_model_sha256 = "model";
  prior.qualification_receipt_sha256 = "wrong-qualification";
  EXPECT_EQ(validate(), "model_hash_mismatch");
  prior.qualification_receipt_sha256 = "qualification";

  prior.dynamic_cost[7] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(validate(), "nonfinite");
}

TEST(LocalRiskGridLayer, validates_local_geometry_identity_and_health)
{
  using bio_nav_fusion::LocalRiskGridLayer;
  bio_nav_interfaces::msg::LocalRiskGrid grid;
  grid.schema_version = "bio_nav_local_risk_grid_v1";
  grid.header.frame_id = "base_link";
  grid.width = 32;
  grid.height = 32;
  grid.resolution = 0.5F;
  grid.origin_x = -8.0F;
  grid.origin_y = -8.0F;
  grid.horizon_s = 0.8F;
  grid.healthy = true;
  grid.reliability = 0.9F;
  grid.ood_probability = 0.1F;
  grid.reset_epoch = 3;
  grid.map_version = "map";
  grid.model_sha256 = "model";
  grid.qualification_receipt_sha256 = "qualification";
  grid.risk.fill(0.0F);
  EXPECT_EQ(
    LocalRiskGridLayer::validateGrid(
      &grid, 0.1, 0.5, 0.6, 0.4, 3, "map", "model", "qualification"),
    "");
  grid.rejection_mask = 4;
  EXPECT_EQ(
    LocalRiskGridLayer::validateGrid(
      &grid, 0.1, 0.5, 0.6, 0.4, 3, "map", "model", "qualification"),
    "risk_unhealthy");
}

TEST(LocalRiskGridLayer, risk_cost_is_strictly_nonlethal)
{
  using bio_nav_fusion::LocalRiskGridLayer;
  EXPECT_EQ(LocalRiskGridLayer::mapRiskCost(0.49F, 0.5F, 80), 0);
  EXPECT_EQ(LocalRiskGridLayer::mapRiskCost(0.5F, 0.5F, 80), 1);
  EXPECT_EQ(LocalRiskGridLayer::mapRiskCost(1.0F, 0.5F, 80), 80);
  EXPECT_LT(LocalRiskGridLayer::mapRiskCost(1.0F, 0.5F, 252), 254);
}
