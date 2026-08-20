#include <array>
#include <limits>

#include "bio_nav_fusion/bio_nav_grid_based.hpp"
#include "bio_nav_fusion/cognitive_obstacle_layer.hpp"
#include "bio_nav_fusion/cognitive_risk_critic.hpp"
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

namespace
{

bio_nav_interfaces::msg::CognitiveObstacleArray obstacleFixture()
{
  bio_nav_interfaces::msg::CognitiveObstacleArray message;
  message.header.frame_id = "base_link";
  message.header.stamp.sec = 10;
  message.sequence = 7;
  message.reset_epoch = 3;
  message.recurrent_session_id = "session";
  message.map_version = "map";
  message.cognitive_tile_id = "tile";
  message.tile_revision = 2;
  message.graph_revision = 4;
  message.schema_version = "bio_nav_cognitive_obstacles_v1";
  message.model_id = "model";
  message.ttl.nanosec = 500000000U;
  message.input_healthy = true;
  message.module2_healthy = true;
  message.observation_valid = true;
  message.trusted_write = true;
  message.reliability = 0.9;
  message.ood_probability = 0.1;
  bio_nav_interfaces::msg::CognitiveObstacle obstacle;
  obstacle.id = "object";
  obstacle.class_id = "unknown_low_obstacle";
  obstacle.pose_xy_m = {1.0, 0.0};
  obstacle.radius_m = 0.2;
  obstacle.height_m = 0.2;
  obstacle.confidence = 0.9;
  obstacle.reliability = 0.9;
  obstacle.ood_probability = 0.1;
  obstacle.position_stddev_m = {0.05, 0.05};
  obstacle.count = 3;
  obstacle.last_seen.sec = 10;
  message.obstacles.push_back(obstacle);
  return message;
}

bio_nav_interfaces::msg::PlanningPrior planningPriorFixture()
{
  bio_nav_interfaces::msg::PlanningPrior prior;
  prior.stamp.sec = 10;
  prior.sequence = 7;
  prior.reset_epoch = 3;
  prior.recurrent_session_id = "session";
  prior.map_version = "map";
  prior.cognitive_tile_id = "tile";
  prior.tile_revision = 2;
  prior.graph_revision = 4;
  prior.model_id = "model";
  prior.schema_version = "bio_nav_planning_prior_v4";
  prior.input_healthy = true;
  prior.module2_healthy = true;
  prior.trusted_write = true;
  prior.context_trusted = true;
  prior.visual_reliability = 0.9F;
  prior.visual_ood_probability = 0.1F;
  prior.novelty_probability = 0.2F;
  prior.context_uncertainty = 0.1F;
  prior.local_direction_ttl.nanosec = 500000000U;
  prior.local_direction_schema_version = "bio_nav_local_direction_prior_v1";
  prior.local_direction_source_sequence = 7;
  prior.local_direction_frame_id = "base_link";
  prior.local_direction_input_healthy = true;
  prior.local_direction_module2_healthy = true;
  prior.local_direction_trusted_write = true;
  prior.local_direction_weights = {0.0, 1.0, 0.0, 0.0, 0.0};
  return prior;
}

}  // namespace

TEST(CognitiveObstacleLayer, strict_gate_and_hard_threshold_are_fail_open)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  EXPECT_FALSE(CognitiveObstacleLayer::modeWritesCostmap("off"));
  EXPECT_FALSE(CognitiveObstacleLayer::modeWritesCostmap("shadow"));
  EXPECT_TRUE(CognitiveObstacleLayer::modeWritesCostmap("active"));
  auto message = obstacleFixture();
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, 6, 0.5, 0.2),
    "");
  message.ood_probability = 0.3;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, 6, 0.5, 0.2),
    "ood");
  message.ood_probability = 0.1;
  EXPECT_EQ(
    CognitiveObstacleLayer::obstacleCost(message.obstacles[0], 80, 0.02, 0.45),
    nav2_costmap_2d::LETHAL_OBSTACLE);
  message.obstacles[0].count = 2;
  const auto soft = CognitiveObstacleLayer::obstacleCost(
    message.obstacles[0], 80, 0.02, 0.45);
  EXPECT_GE(soft, 1U);
  EXPECT_LE(soft, 80U);
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10600000000LL, identity, 6, 0.5, 0.2),
    "stale");
  message.header.stamp.sec = 10;
  message.recurrent_session_id = "wrong";
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, 6, 0.5, 0.2),
    "identity");
}

TEST(CognitiveRiskCritic, nearer_and_more_directionally_deviant_cost_more)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::vector<Critic::ObstacleSample> obstacles{{1.0, 0.0, 0.2, 1.0}};
  const std::array<double, 5> east{0.0, 1.0, 0.0, 0.0, 0.0};
  const std::vector<std::array<double, 3>> near{
    {0.0, 0.0, 0.0}, {0.9, 0.0, 0.0}};
  const std::vector<std::array<double, 3>> far{
    {0.0, 1.5, 0.0}, {0.9, 1.5, 0.0}};
  const std::vector<std::array<double, 3>> west{
    {0.0, 1.5, M_PI}, {-0.9, 1.5, M_PI}};
  const auto near_cost = Critic::trajectoryScore(
    near, obstacles, east, 0.0, 0.2, 0.1, 4.0, 1.0, 0.5, 0.5);
  const auto far_cost = Critic::trajectoryScore(
    far, obstacles, east, 0.0, 0.2, 0.1, 4.0, 1.0, 0.5, 0.5);
  const auto west_cost = Critic::trajectoryScore(
    west, {}, east, 0.0, 0.2, 0.1, 4.0, 1.0, 0.5, 0.5);
  const auto east_cost = Critic::trajectoryScore(
    far, {}, east, 0.0, 0.2, 0.1, 4.0, 1.0, 0.5, 0.5);
  EXPECT_GT(near_cost, far_cost);
  EXPECT_GT(west_cost, east_cost);
}

TEST(CognitiveRiskCritic, base_direction_uses_robot_yaw_and_stay_has_no_bias)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::array<double, 5> east{0.0, 1.0, 0.0, 0.0, 0.0};
  const std::array<double, 5> zero{};
  const std::array<double, 5> stay{0.0, 0.0, 0.0, 0.0, 1.0};
  const std::vector<std::array<double, 3>> north{
    {0.0, 0.0, 0.5 * M_PI}, {0.0, 1.0, 0.5 * M_PI}};
  const std::vector<std::array<double, 3>> east_path{
    {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const auto rotated_match = Critic::trajectoryScore(
    north, {}, east, 0.5 * M_PI, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  const auto rotated_miss = Critic::trajectoryScore(
    east_path, {}, east, 0.5 * M_PI, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  EXPECT_LT(rotated_match, rotated_miss);
  EXPECT_DOUBLE_EQ(Critic::trajectoryScore(
      north, {}, zero, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(Critic::trajectoryScore(
      north, {}, stay, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0), 0.0);
}

TEST(CognitiveRiskCritic, stale_or_ood_inputs_score_zero_by_rejection)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto obstacles = obstacleFixture();
  auto prior = planningPriorFixture();
  EXPECT_EQ(Critic::validateInputs(
      &obstacles, &prior, 10100000000LL, 0.5, 0.2), "");
  EXPECT_EQ(Critic::validateDirectionPrior(prior), "");
  EXPECT_EQ(Critic::validateInputs(
      &obstacles, &prior, 10600000000LL, 0.5, 0.2), "stale");
  prior.visual_ood_probability = 0.8F;
  EXPECT_EQ(Critic::validateInputs(
      &obstacles, &prior, 10100000000LL, 0.5, 0.2), "ood");

  prior = planningPriorFixture();
  prior.local_direction_frame_id = "module2_canvas";
  prior.local_direction_trusted_write = false;
  EXPECT_EQ(Critic::validateDirectionPrior(prior), "direction_frame");
}
