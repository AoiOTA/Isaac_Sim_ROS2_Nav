#include <array>
#include <limits>

#include "bio_nav_fusion/bio_nav_grid_based.hpp"
#include "bio_nav_fusion/cognitive_risk_layer.hpp"
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
