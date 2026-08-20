#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"

TEST(BioNavFusionPlugins, risk_layer_does_not_preload_stock_smac_factory)
{
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> costmap_loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  const auto risk_layer = costmap_loader.createUniqueInstance(
    "bio_nav_fusion::CognitiveRiskLayer");
  ASSERT_NE(risk_layer, nullptr);
  const auto obstacle_layer = costmap_loader.createUniqueInstance(
    "bio_nav_fusion::CognitiveObstacleLayer");
  ASSERT_NE(obstacle_layer, nullptr);
  const auto local_risk_layer = costmap_loader.createUniqueInstance(
    "bio_nav_fusion::LocalRiskGridLayer");
  ASSERT_NE(local_risk_layer, nullptr);
  const auto reachability_observer = costmap_loader.createUniqueInstance(
    "bio_nav_fusion::ReachabilityObserverLayer");
  ASSERT_NE(reachability_observer, nullptr);

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planner_loader(
    "nav2_core", "nav2_core::GlobalPlanner");
  const auto stock_planner = planner_loader.createUniqueInstance(
    "nav2_smac_planner::SmacPlanner2D");
  EXPECT_NE(stock_planner, nullptr);

  pluginlib::ClassLoader<mppi::critics::CriticFunction> critic_loader(
    "nav2_mppi_controller", "mppi::critics::CriticFunction");
  const auto cognitive_critic = critic_loader.createUniqueInstance(
    "mppi::critics::CognitiveRiskCritic");
  EXPECT_NE(cognitive_critic, nullptr);
}
