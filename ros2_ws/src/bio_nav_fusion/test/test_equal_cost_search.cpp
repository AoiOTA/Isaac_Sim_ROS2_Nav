#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include "bio_nav_fusion/bio_nav_grid_based.hpp"
#include "bio_nav_fusion/cognitive_obstacle_layer.hpp"
#include "bio_nav_fusion/cognitive_risk_critic.hpp"
#include "bio_nav_fusion/cognitive_risk_layer.hpp"
#include "bio_nav_fusion/reachability_observer_layer.hpp"
#include "bio_nav_fusion/local_risk_grid_layer.hpp"
#include "gtest/gtest.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace bio_nav_fusion
{

class CognitiveRiskCriticTestPeer
{
public:
  static void configure(
    CognitiveRiskCritic & critic,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap)
  {
    critic.enabled_ = true;
    critic.mode_ = "active";
    critic.parent_ = costmap;
    critic.costmap_ros_ = costmap;
    critic.maximum_age_s_ = 0.5;
    critic.maximum_ood_probability_ = 0.2;
    critic.obstacle_weight_ = 1.0F;
    critic.direction_weight_ = 0.0F;
    critic.novelty_weight_ = 0.0F;
    critic.uncertainty_weight_ = 0.0F;
    critic.obstacles_.reset();
    critic.prior_.reset();
    critic.expected_ = CognitiveObstacleLayer::Identity{};
    critic.accepted_.reset();
    critic.identity_bound_ = false;
    critic.route_context_ = CognitiveRiskCritic::RouteContext{};
    critic.route_identity_ = CognitiveObstacleLayer::Identity{};
    critic.route_context_bound_ = false;
    critic.pending_rebind_identity_ = CognitiveObstacleLayer::Identity{};
    critic.pending_rebind_ = false;
    critic.last_rejected_offer_ = CognitiveRiskCritic::RejectedOffer{};
    critic.last_status_sequence_ = 0;
    critic.last_status_applied_ = false;
    critic.last_status_reason_.clear();
  }

  static void setInputs(
    CognitiveRiskCritic & critic,
    const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr & obstacles,
    const bio_nav_interfaces::msg::PlanningPrior::SharedPtr & prior)
  {
    if (prior) {
      critic.priorCallback(prior);
    }
    if (obstacles) {
      critic.obstacleCallback(obstacles);
    }
  }

  static void offerObstacle(
    CognitiveRiskCritic & critic,
    const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr & obstacles)
  {
    critic.obstacleCallback(obstacles);
  }

  static void offerPrior(
    CognitiveRiskCritic & critic,
    const bio_nav_interfaces::msg::PlanningPrior::SharedPtr & prior)
  {
    critic.priorCallback(prior);
  }

  static CognitiveObstacleLayer::Identity identity(CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.expected_;
  }

  static CognitiveObstacleLayer::AcceptanceCursor cursor(CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.accepted_;
  }

  static bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr obstacles(
    CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.obstacles_;
  }

  static void useDirectionOnly(CognitiveRiskCritic & critic)
  {
    critic.obstacle_weight_ = 0.0F;
    critic.direction_weight_ = 1.0F;
  }

  static void useAllComponents(CognitiveRiskCritic & critic)
  {
    critic.obstacle_weight_ = 1.0F;
    critic.direction_weight_ = 1.0F;
    critic.novelty_weight_ = 1.0F;
    critic.uncertainty_weight_ = 1.0F;
  }

  static void setNonObstacleWeights(
    CognitiveRiskCritic & critic, float direction, float novelty,
    float uncertainty)
  {
    critic.direction_weight_ = direction;
    critic.novelty_weight_ = novelty;
    critic.uncertainty_weight_ = uncertainty;
  }

  static void useContextOnly(CognitiveRiskCritic & critic)
  {
    critic.obstacle_weight_ = 0.0F;
    critic.direction_weight_ = 0.0F;
    critic.novelty_weight_ = 1.0F;
    critic.uncertainty_weight_ = 0.0F;
  }

  static void useZeroWeights(CognitiveRiskCritic & critic)
  {
    critic.obstacle_weight_ = 0.0F;
    critic.direction_weight_ = 0.0F;
    critic.novelty_weight_ = 0.0F;
    critic.uncertainty_weight_ = 0.0F;
  }

  static std::string appliedStatus(
    const std::string & prior_reason, const std::string & context_reason,
    const std::string & direction_reason, bool obstacle_applied = true,
    bool novelty_applied = false, bool uncertainty_applied = false,
    bool direction_applied = false)
  {
    return CognitiveRiskCritic::appliedStatus(
      prior_reason, context_reason, direction_reason, obstacle_applied,
      novelty_applied, uncertainty_applied, direction_applied);
  }

  static CognitiveRiskCritic::RejectedOffer lastRejected(CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.last_rejected_offer_;
  }

  static bool lastStatusApplied(CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.last_status_applied_;
  }

  static uint64_t lastStatusSequence(CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.last_status_sequence_;
  }

  static std::string lastStatusReason(CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.last_status_reason_;
  }
};

}  // namespace bio_nav_fusion

namespace
{

class CriticTestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  CriticTestCostmap()
  : nav2_costmap_2d::Costmap2DROS("cognitive_risk_critic_test_costmap")
  {
    global_frame_ = "map";
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  }
};

}  // namespace

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
  message.validation_stamp.sec = 10;
  message.validation_ttl.nanosec = 500000000U;
  message.source_odom_stamp.sec = 10;
  message.validation_odom_stamp.sec = 10;
  message.validation_mode =
    bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_FRESH;
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
  obstacle.motion_class = bio_nav_interfaces::msg::CognitiveObstacle::MOTION_UNKNOWN;
  message.obstacles.push_back(obstacle);
  return message;
}

bio_nav_interfaces::msg::CognitiveObstacleArray staticRevalidatedObstacleFixture()
{
  auto message = obstacleFixture();
  message.validation_stamp.sec = 11;
  message.source_age.sec = 1;
  message.validation_ttl.nanosec = 500000000U;
  message.validation_odom_stamp.sec = 11;
  message.validation_mode =
    bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_STATIC_DEPTH_REVALIDATED;
  message.validation_sensor_mask =
    bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_SENSOR_DEPTH;
  message.obstacles[0].motion_class =
    bio_nav_interfaces::msg::CognitiveObstacle::MOTION_STATIC;
  message.obstacles[0].static_confirmed = true;
  return message;
}

bio_nav_interfaces::msg::CognitiveObstacleArray staticRevalidatedObstacleWithAge(
  int32_t age_sec, uint32_t age_nanosec)
{
  auto message = staticRevalidatedObstacleFixture();
  message.source_age.sec = age_sec;
  message.source_age.nanosec = age_nanosec;
  message.validation_stamp.sec = message.header.stamp.sec + age_sec;
  message.validation_stamp.nanosec = age_nanosec;
  message.validation_odom_stamp = message.validation_stamp;
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
  prior.observation_valid = true;
  prior.trusted_write = true;
  prior.context_trusted = true;
  prior.visual_reliability = 0.9F;
  prior.visual_ood_probability = 0.1F;
  prior.novelty_probability = 0.2F;
  prior.context_uncertainty = 0.1F;
  prior.local_direction_ttl.nanosec = 500000000U;
  prior.local_direction_schema_version = "bio_nav_local_direction_prior_v1";
  prior.local_direction_graph_id = "route-graph";
  prior.local_direction_source_sequence = 7;
  prior.source_physical_graph_id = "physical-graph";
  prior.source_physical_graph_revision = 5;
  prior.topology_revision = 6;
  prior.local_direction_frame_id = "base_link";
  prior.local_direction_input_healthy = true;
  prior.local_direction_module2_healthy = true;
  prior.local_direction_trusted_write = true;
  prior.local_direction_weights = {0.0, 1.0, 0.0, 0.0, 0.0};
  return prior;
}

bio_nav_interfaces::msg::PlanningPrior productionV310PriorFixture()
{
  auto prior = planningPriorFixture();
  prior.schema_version = "bio_nav_planning_prior_v310";
  prior.context_trusted = false;
  prior.local_direction_frame_id = "module2_canvas";
  prior.local_direction_trusted_write = false;
  return prior;
}

std::string layerObstacleVerdict(
  const bio_nav_interfaces::msg::CognitiveObstacleArray & obstacles,
  const bio_nav_interfaces::msg::PlanningPrior & prior, int64_t now_ns)
{
  const bio_nav_fusion::CognitiveObstacleLayer::Identity identity{
    prior.reset_epoch, prior.recurrent_session_id, prior.map_version,
    prior.cognitive_tile_id, prior.tile_revision, prior.graph_revision,
    prior.model_id};
  return bio_nav_fusion::CognitiveObstacleLayer::validateMessage(
    obstacles, now_ns, identity,
    bio_nav_fusion::CognitiveObstacleLayer::AcceptanceCursor{}, 0.5, 0.2);
}

builtin_interfaces::msg::Time stampFromNs(int64_t stamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
  return stamp;
}

builtin_interfaces::msg::Duration durationFromNs(int64_t duration_ns)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(duration_ns / 1000000000LL);
  duration.nanosec = static_cast<uint32_t>(duration_ns % 1000000000LL);
  return duration;
}

void retimeFreshObstacle(
  bio_nav_interfaces::msg::CognitiveObstacleArray & obstacles, int64_t source_ns)
{
  obstacles.header.stamp = stampFromNs(source_ns);
  obstacles.validation_stamp = obstacles.header.stamp;
  obstacles.source_age = durationFromNs(0);
  obstacles.source_odom_stamp = obstacles.header.stamp;
  obstacles.validation_odom_stamp = obstacles.header.stamp;
  obstacles.obstacles[0].last_seen = obstacles.header.stamp;
}

void retimeFresh(
  bio_nav_interfaces::msg::CognitiveObstacleArray & obstacles,
  bio_nav_interfaces::msg::PlanningPrior & prior, int64_t source_ns)
{
  retimeFreshObstacle(obstacles, source_ns);
  prior.stamp = obstacles.header.stamp;
}

void retimeStatic(
  bio_nav_interfaces::msg::CognitiveObstacleArray & obstacles,
  int64_t source_ns, int64_t validation_ns)
{
  obstacles.header.stamp = stampFromNs(source_ns);
  obstacles.validation_stamp = stampFromNs(validation_ns);
  obstacles.source_age = durationFromNs(validation_ns - source_ns);
  obstacles.source_odom_stamp = obstacles.header.stamp;
  obstacles.validation_odom_stamp = obstacles.validation_stamp;
  obstacles.obstacles[0].last_seen = obstacles.header.stamp;
}

bool addTransform(
  const std::shared_ptr<CriticTestCostmap> & costmap, int64_t stamp_ns,
  double translation_x, double yaw = 0.0)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = stampFromNs(stamp_ns);
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = translation_x;
  transform.transform.rotation.z = std::sin(0.5 * yaw);
  transform.transform.rotation.w = std::cos(0.5 * yaw);
  return costmap->getTfBuffer()->setTransform(transform, "critic_test");
}

float scoreAt(
  bio_nav_fusion::CognitiveRiskCritic & critic, float x, float y,
  float yaw = 0.0F)
{
  mppi::models::State state;
  mppi::models::Trajectories trajectories;
  trajectories.reset(1U, 1U);
  trajectories.x(0, 0) = x;
  trajectories.y(0, 0) = y;
  trajectories.yaws(0, 0) = yaw;
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs{0.0F};
  float model_dt = 0.05F;
  mppi::CriticData data{
    state, trajectories, path, goal, costs, model_dt, false, nullptr,
    std::shared_ptr<mppi::MotionModel>{}, std::nullopt, std::nullopt};
  critic.score(data);
  return costs(0);
}

std::shared_ptr<CriticTestCostmap> makeCriticTestCostmap()
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  return std::make_shared<CriticTestCostmap>();
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
    "validation_stale");
  message.header.stamp.sec = 10;
  message.recurrent_session_id = "wrong";
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, 6, 0.5, 0.2),
    "identity");
}

TEST(CognitiveObstacleLayer, consumer_identity_distinguishes_fake_costmap_namespaces)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  const std::string layer = "cognitive_obstacle_layer";
  const auto global = CognitiveObstacleLayer::resolveConsumerId(
    "/global_costmap/global_costmap", layer);
  const auto local = CognitiveObstacleLayer::resolveConsumerId(
    "/local_costmap/local_costmap", layer);
  EXPECT_EQ(global, "/global_costmap/global_costmap:cognitive_obstacle_layer");
  EXPECT_EQ(local, "/local_costmap/local_costmap:cognitive_obstacle_layer");
  EXPECT_NE(global, local);
  EXPECT_EQ(
    global,
    CognitiveObstacleLayer::resolveConsumerId(
      "/global_costmap/global_costmap", layer));
}

TEST(CognitiveObstacleLayer, consumer_identity_override_and_empty_fallback_are_stable)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  EXPECT_EQ(
    CognitiveObstacleLayer::resolveConsumerId(
      "/global_costmap/global_costmap", "cognitive_obstacle_layer", "global"),
    "global");
  EXPECT_EQ(
    CognitiveObstacleLayer::resolveConsumerId(
      "/global_costmap/global_costmap", "cognitive_obstacle_layer", ""),
    "/global_costmap/global_costmap:cognitive_obstacle_layer");
  EXPECT_EQ(
    CognitiveObstacleLayer::resolveConsumerId("", "", ""),
    "/unknown_costmap:cognitive_obstacle_layer");
}

TEST(CognitiveObstacleLayer, static_depth_revalidation_requires_exact_dual_timeline)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  auto message = staticRevalidatedObstacleFixture();
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity, 6, 0.5, 0.2),
    "");

  auto changed = message;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11600000000LL, identity, 6, 0.5, 0.2),
    "validation_stale");

  changed = message;
  changed.header.stamp.sec = 8;
  changed.source_age.sec = 3;
  changed.source_odom_stamp.sec = 8;
  changed.obstacles[0].last_seen.sec = 8;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "");

  changed = message;
  changed.source_age.nanosec = 1U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "source_age");

  changed = message;
  changed.header.stamp = changed.validation_stamp;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "source_age");

  changed = message;
  changed.validation_odom_stamp.sec = 10;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "odom_time");
}

TEST(CognitiveObstacleLayer, static_source_age_accepts_up_to_five_seconds_only)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  const CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};

  const auto age_1_99 = staticRevalidatedObstacleWithAge(1, 990000000U);
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      age_1_99, 12090000000LL, identity, 6, 0.5, 0.2),
    "");

  const auto age_2_2 = staticRevalidatedObstacleWithAge(2, 200000000U);
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      age_2_2, 12300000000LL, identity, 6, 0.5, 0.2),
    "");

  const auto age_4_9 = staticRevalidatedObstacleWithAge(4, 900000000U);
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      age_4_9, 15000000000LL, identity, 6, 0.5, 0.2),
    "");

  const auto age_5_01 = staticRevalidatedObstacleWithAge(5, 10000000U);
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      age_5_01, 15110000000LL, identity, 6, 0.5, 0.2),
    "source_age");
}

TEST(CognitiveObstacleLayer, fresh_source_age_remains_exactly_zero)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  const CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  auto message = obstacleFixture();
  message.source_age.sec = 2;
  message.source_age.nanosec = 200000000U;
  message.validation_stamp.sec = 12;
  message.validation_stamp.nanosec = 200000000U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 12300000000LL, identity, 6, 0.5, 0.2),
    "fresh_mismatch");
}

TEST(CognitiveObstacleLayer, fresh_accepts_zero_odom_and_rejects_nonzero_mismatch)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  const CognitiveObstacleLayer::AcceptanceCursor no_prior;
  auto message = obstacleFixture();
  message.source_odom_stamp.sec = 0;
  message.validation_odom_stamp.sec = 0;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, no_prior, 0.5, 0.2),
    "");

  message.source_odom_stamp.sec = 10;
  message.validation_odom_stamp.sec = 10;
  message.validation_odom_stamp.nanosec = 1U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, no_prior, 0.5, 0.2),
    "odom_time");

  message = obstacleFixture();
  message.validation_sensor_mask =
    bio_nav_interfaces::msg::CognitiveObstacleArray::VALIDATION_SENSOR_DEPTH;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, no_prior, 0.5, 0.2),
    "fresh_mismatch");
}

TEST(CognitiveObstacleLayer, static_revalidation_requires_positive_odom_endpoints)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  auto message = staticRevalidatedObstacleFixture();
  message.source_odom_stamp.sec = 0;
  message.validation_odom_stamp.sec = 0;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity,
      CognitiveObstacleLayer::AcceptanceCursor{}, 0.5, 0.2),
    "odom_time");
}

TEST(CognitiveObstacleLayer, same_source_static_validation_refresh_is_monotonic)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  CognitiveObstacleLayer::AcceptanceCursor accepted;
  auto fresh = obstacleFixture();
  fresh.source_odom_stamp.sec = 0;
  fresh.validation_odom_stamp.sec = 0;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      fresh, 10100000000LL, identity, accepted, 0.5, 0.2),
    "");
  CognitiveObstacleLayer::recordAccepted(fresh, accepted);

  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      fresh, 10100000000LL, identity, accepted, 0.5, 0.2),
    "sequence");

  auto refresh = staticRevalidatedObstacleFixture();
  refresh.obstacles[0].pose_xy_m[0] = 1.5;
  refresh.obstacles[0].count = 2;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      refresh, 11100000000LL, identity, accepted, 0.5, 0.2),
    "");
  const auto refreshed_cost = CognitiveObstacleLayer::obstacleCost(
    refresh.obstacles[0], 80, 0.02, 0.45);
  EXPECT_LT(refreshed_cost, nav2_costmap_2d::LETHAL_OBSTACLE);
  CognitiveObstacleLayer::recordAccepted(refresh, accepted);
  EXPECT_EQ(accepted.source_sequence, 7U);
  EXPECT_EQ(accepted.source_stamp_ns, 10000000000LL);
  EXPECT_EQ(accepted.validation_stamp_ns, 11000000000LL);

  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      refresh, 11100000000LL, identity, accepted, 0.5, 0.2),
    "validation_regression");

  auto backward = refresh;
  backward.validation_stamp.sec = 10;
  backward.validation_stamp.nanosec = 500000000U;
  backward.source_age.sec = 0;
  backward.source_age.nanosec = 500000000U;
  backward.validation_odom_stamp.sec = 10;
  backward.validation_odom_stamp.nanosec = 500000000U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      backward, 10600000000LL, identity, accepted, 0.5, 0.2),
    "validation_regression");

  auto changed_source = refresh;
  changed_source.header.stamp.nanosec = 100000000U;
  changed_source.validation_stamp.nanosec = 200000000U;
  changed_source.source_age.nanosec = 100000000U;
  changed_source.source_odom_stamp.nanosec = 100000000U;
  changed_source.validation_odom_stamp.nanosec = 200000000U;
  changed_source.obstacles[0].last_seen.nanosec = 100000000U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed_source, 11300000000LL, identity, accepted, 0.5, 0.2),
    "source_mismatch");

  auto changed_identity = refresh;
  changed_identity.validation_stamp.nanosec = 100000000U;
  changed_identity.source_age.nanosec = 100000000U;
  changed_identity.validation_odom_stamp.nanosec = 100000000U;
  changed_identity.map_version = "other";
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed_identity, 11200000000LL, identity, accepted, 0.5, 0.2),
    "identity");

  auto regressed_source = obstacleFixture();
  regressed_source.sequence = 8;
  regressed_source.header.stamp.sec = 9;
  regressed_source.validation_stamp.sec = 9;
  regressed_source.source_odom_stamp.sec = 0;
  regressed_source.validation_odom_stamp.sec = 0;
  regressed_source.obstacles[0].last_seen.sec = 9;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      regressed_source, 9100000000LL, identity, accepted, 0.5, 0.2),
    "source_regression");

  accepted.reset();
  EXPECT_FALSE(accepted.valid);
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      fresh, 10100000000LL, identity, accepted, 0.5, 0.2),
    "");
}

TEST(CognitiveObstacleLayer, static_depth_revalidation_rejects_unconfirmed_items_and_mask)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  auto message = staticRevalidatedObstacleFixture();
  message.validation_sensor_mask = 0U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity, 6, 0.5, 0.2),
    "validation_sensor");

  message = staticRevalidatedObstacleFixture();
  message.obstacles[0].motion_class =
    bio_nav_interfaces::msg::CognitiveObstacle::MOTION_DYNAMIC;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity, 6, 0.5, 0.2),
    "static_confirmation");

  message.obstacles[0].motion_class =
    bio_nav_interfaces::msg::CognitiveObstacle::MOTION_UNKNOWN;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity, 6, 0.5, 0.2),
    "static_confirmation");

  message.obstacles[0].motion_class =
    bio_nav_interfaces::msg::CognitiveObstacle::MOTION_STATIC;
  message.obstacles[0].static_confirmed = false;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity, 6, 0.5, 0.2),
    "static_confirmation");
}

TEST(CognitiveObstacleLayer, future_tolerance_and_identity_fail_open)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  auto message = obstacleFixture();
  message.header.stamp.nanosec = 50000000U;
  message.validation_stamp.nanosec = 50000000U;
  message.source_odom_stamp.nanosec = 50000000U;
  message.validation_odom_stamp.nanosec = 50000000U;
  message.obstacles[0].last_seen.nanosec = 50000000U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10000000000LL, identity, 6, 0.5, 0.2),
    "");
  message.header.stamp.nanosec = 50000001U;
  message.validation_stamp.nanosec = 50000001U;
  message.source_odom_stamp.nanosec = 50000001U;
  message.validation_odom_stamp.nanosec = 50000001U;
  message.obstacles[0].last_seen.nanosec = 50000001U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10000000000LL, identity, 6, 0.5, 0.2),
    "validation_stale");

  message = obstacleFixture();
  message.map_version = "other";
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 10100000000LL, identity, 6, 0.5, 0.2),
    "identity");
}

TEST(CognitiveObstacleLayer, shadow_never_raises_and_active_uses_max_merge)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  EXPECT_EQ(CognitiveObstacleLayer::mergeCellCost("shadow", 40U, 80U), 40U);
  EXPECT_EQ(CognitiveObstacleLayer::mergeCellCost("off", 40U, 80U), 40U);
  EXPECT_EQ(CognitiveObstacleLayer::mergeCellCost("active", 40U, 80U), 80U);
  EXPECT_EQ(CognitiveObstacleLayer::mergeCellCost("active", 90U, 80U), 90U);
}

TEST(CognitiveObstacleLayer, tf_failure_has_explicit_zero_raise_fail_open_contract)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  EXPECT_STREQ(CognitiveObstacleLayer::tfFailureReason(), "tf");
  EXPECT_EQ(CognitiveObstacleLayer::mergeCellCost("active", 40U, 0U), 40U);
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

TEST(CognitiveRiskCritic, validation_distinguishes_obstacle_and_prior_rejections)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto obstacles = obstacleFixture();
  auto prior = planningPriorFixture();
  EXPECT_EQ(Critic::validateInputs(
      &obstacles, &prior, 10100000000LL, 0.5, 0.2), "");
  EXPECT_EQ(Critic::validateDirectionPrior(prior), "");
  EXPECT_EQ(Critic::validateInputs(
      &obstacles, &prior, 10600000000LL, 0.5, 0.2), "validation_stale");
  prior.visual_ood_probability = 0.8F;
  EXPECT_EQ(Critic::validateInputs(
      &obstacles, &prior, 10100000000LL, 0.5, 0.2), "prior_ood");

  prior = planningPriorFixture();
  prior.local_direction_frame_id = "module2_canvas";
  prior.local_direction_trusted_write = false;
  EXPECT_EQ(Critic::validateDirectionPrior(prior), "direction_frame");
}

TEST(CognitiveRiskCritic, static_depth_revalidation_does_not_retime_source_prior)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto obstacles = staticRevalidatedObstacleFixture();
  auto prior = planningPriorFixture();
  const int64_t now_ns = 11100000000LL;

  const auto layer_reason = layerObstacleVerdict(obstacles, prior, now_ns);
  EXPECT_EQ(layer_reason, "");
  EXPECT_EQ(Critic::validatePriorComponents(
      &obstacles, &prior, now_ns, 0.5, 0.2), "prior_stale");
}

TEST(CognitiveRiskCritic, static_revalidation_survives_stale_missing_and_mismatched_prior)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();
  const int64_t validation_ns = costmap->now().nanoseconds() - 10000000LL;
  const int64_t source_ns = validation_ns - 1000000000LL;
  ASSERT_TRUE(addTransform(costmap, validation_ns, 0.0));

  auto obstacles = staticRevalidatedObstacleFixture();
  auto prior = productionV310PriorFixture();
  retimeStatic(obstacles, source_ns, validation_ns);
  prior.stamp = stampFromNs(source_ns);
  EXPECT_EQ(Critic::validatePriorComponents(
      &obstacles, &prior, costmap->now().nanoseconds(), 0.5, 0.2),
    "prior_stale");

  Critic obstacle_only;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(obstacle_only, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    obstacle_only,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  const auto obstacle_cost = scoreAt(obstacle_only, 1.0F, 0.0F, static_cast<float>(M_PI));
  EXPECT_GT(obstacle_cost, 1.0F);

  Critic stale_prior;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(stale_prior, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useAllComponents(stale_prior);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    stale_prior,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_FLOAT_EQ(
    scoreAt(stale_prior, 1.0F, 0.0F, static_cast<float>(M_PI)), obstacle_cost);

  auto latest_mismatch = prior;
  latest_mismatch.stamp = stampFromNs(validation_ns);
  latest_mismatch.sequence += 1U;
  latest_mismatch.local_direction_source_sequence = latest_mismatch.sequence;
  Critic mismatched_prior;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(mismatched_prior, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useAllComponents(mismatched_prior);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    mismatched_prior,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(latest_mismatch));
  EXPECT_FLOAT_EQ(
    scoreAt(mismatched_prior, 1.0F, 0.0F, static_cast<float>(M_PI)), obstacle_cost);

  Critic missing_prior;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(missing_prior, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useAllComponents(missing_prior);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    missing_prior,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles), nullptr);
  EXPECT_FLOAT_EQ(
    scoreAt(missing_prior, 1.0F, 0.0F, static_cast<float>(M_PI)), obstacle_cost);

  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::appliedStatus(
      "prior_stale", "", ""),
    "cost_delta_applied=true;obstacle_applied=true;prior_suppressed=prior_stale"
    ";context_suppressed=prior_stale;novelty_suppressed=prior_stale"
    ";uncertainty_suppressed=prior_stale;direction_suppressed=prior_stale");
  const auto component_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::appliedStatus(
    "", "context_untrusted", "direction_frame");
  EXPECT_NE(component_status.find("obstacle_applied=true"), std::string::npos);
  EXPECT_NE(
    component_status.find("context_suppressed=context_untrusted"),
    std::string::npos);
  EXPECT_NE(
    component_status.find("direction_suppressed=direction_frame"),
    std::string::npos);
}

TEST(CognitiveRiskCritic, fresh_original_pair_enables_legal_prior_components)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();
  const int64_t live_ns = costmap->now().nanoseconds() - 10000000LL;
  ASSERT_TRUE(addTransform(costmap, live_ns, 0.0));
  auto obstacles = obstacleFixture();
  auto prior = planningPriorFixture();
  retimeFresh(obstacles, prior, live_ns);

  Critic obstacle_only;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(obstacle_only, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    obstacle_only,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  const auto obstacle_cost = scoreAt(
    obstacle_only, 1.0F, 0.0F, static_cast<float>(M_PI));

  Critic complete;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(complete, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useAllComponents(complete);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    complete,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_NEAR(
    scoreAt(complete, 1.0F, 0.0F, static_cast<float>(M_PI)),
    obstacle_cost + 1.0F + prior.novelty_probability + prior.context_uncertainty,
    1.0e-5F);
}

TEST(CognitiveRiskCritic, expired_static_validation_and_live_source_fail_open)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto prior = planningPriorFixture();

  auto obstacles = staticRevalidatedObstacleFixture();
  prior.stamp.sec = 11;
  prior.stamp.nanosec = 600000000U;
  const int64_t static_now_ns = 11600000000LL;
  auto layer_reason = layerObstacleVerdict(obstacles, prior, static_now_ns);
  auto critic_reason = Critic::validateInputs(
    &obstacles, &prior, static_now_ns, 0.5, 0.2);
  EXPECT_EQ(layer_reason, "validation_stale");
  EXPECT_EQ(critic_reason, layer_reason);

  obstacles = obstacleFixture();
  prior.stamp.sec = 10;
  prior.stamp.nanosec = 600000000U;
  const int64_t live_now_ns = 10600000000LL;
  layer_reason = layerObstacleVerdict(obstacles, prior, live_now_ns);
  critic_reason = Critic::validateInputs(
    &obstacles, &prior, live_now_ns, 0.5, 0.2);
  EXPECT_EQ(layer_reason, "validation_stale");
  EXPECT_EQ(critic_reason, layer_reason);
}

TEST(CognitiveRiskCritic, obstacle_identity_ood_and_trust_match_layer_fail_open)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const int64_t now_ns = 10100000000LL;
  const auto prior = planningPriorFixture();

  auto obstacles = obstacleFixture();
  obstacles.map_version = "wrong";
  EXPECT_EQ(layerObstacleVerdict(obstacles, prior, now_ns), "identity");
  EXPECT_EQ(
    Critic::validateInputs(&obstacles, &prior, now_ns, 0.5, 0.2),
    layerObstacleVerdict(obstacles, prior, now_ns));

  obstacles = obstacleFixture();
  obstacles.ood_probability = 0.3;
  EXPECT_EQ(layerObstacleVerdict(obstacles, prior, now_ns), "ood");
  EXPECT_EQ(
    Critic::validateInputs(&obstacles, &prior, now_ns, 0.5, 0.2),
    layerObstacleVerdict(obstacles, prior, now_ns));

  obstacles = obstacleFixture();
  obstacles.trusted_write = false;
  EXPECT_EQ(layerObstacleVerdict(obstacles, prior, now_ns), "untrusted");
  EXPECT_EQ(
    Critic::validateInputs(&obstacles, &prior, now_ns, 0.5, 0.2),
    layerObstacleVerdict(obstacles, prior, now_ns));
}

TEST(CognitiveRiskCritic, score_uses_validation_tf_for_static_and_source_tf_for_live)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();

  const int64_t validation_ns = costmap->now().nanoseconds() - 20000000LL;
  const int64_t source_ns = validation_ns - 1000000000LL;
  ASSERT_TRUE(addTransform(costmap, source_ns, 0.0));
  ASSERT_TRUE(addTransform(costmap, validation_ns, 10.0, 0.5 * M_PI));
  auto static_obstacles = staticRevalidatedObstacleFixture();
  retimeStatic(static_obstacles, source_ns, validation_ns);
  Critic static_critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(static_critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    static_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(static_obstacles), nullptr);

  const auto validation_frame_cost = scoreAt(static_critic, 10.0F, 1.0F);
  EXPECT_GT(validation_frame_cost, 1.0F);
  EXPECT_GT(scoreAt(static_critic, 10.0F, 1.0F), 1.0F);
  EXPECT_LT(scoreAt(static_critic, 1.0F, 0.0F), 0.01F);

  const int64_t live_ns = costmap->now().nanoseconds() - 10000000LL;
  ASSERT_TRUE(addTransform(costmap, live_ns, -5.0));
  auto live_obstacles = obstacleFixture();
  auto live_prior = planningPriorFixture();
  retimeFresh(live_obstacles, live_prior, live_ns);
  Critic live_critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(live_critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    live_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(live_obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(live_prior));

  EXPECT_GT(scoreAt(live_critic, -4.0F, 0.0F), 1.0F);
  EXPECT_GT(scoreAt(live_critic, -4.0F, 0.0F), 1.0F);
  EXPECT_LT(scoreAt(live_critic, 1.0F, 0.0F), 0.01F);
}

TEST(CognitiveRiskCritic, score_fails_open_for_missing_expired_bad_ood_and_untrusted)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();
  const auto fresh_ns = costmap->now().nanoseconds() - 10000000LL;

  Critic missing_critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(missing_critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    missing_critic, nullptr,
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(planningPriorFixture()));
  EXPECT_FLOAT_EQ(scoreAt(missing_critic, 1.0F, 0.0F), 0.0F);

  auto expired_obstacles = staticRevalidatedObstacleFixture();
  auto expired_prior = planningPriorFixture();
  retimeStatic(
    expired_obstacles, fresh_ns - 2000000000LL,
    fresh_ns - 1000000000LL);
  expired_prior.stamp = stampFromNs(fresh_ns);
  Critic expired_critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(expired_critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    expired_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(expired_obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(expired_prior));
  EXPECT_FLOAT_EQ(scoreAt(expired_critic, 1.0F, 0.0F), 0.0F);

  const auto expect_zero = [&](bio_nav_interfaces::msg::CognitiveObstacleArray message) {
      auto prior = planningPriorFixture();
      retimeFresh(message, prior, fresh_ns);
      Critic critic;
      bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(critic, costmap);
      bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
        critic,
        std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(message),
        std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
      EXPECT_FLOAT_EQ(scoreAt(critic, 1.0F, 0.0F), 0.0F);
    };

  auto bad = obstacleFixture();
  bad.obstacles[0].pose_xy_m[0] = std::numeric_limits<double>::quiet_NaN();
  expect_zero(bad);
  auto ood = obstacleFixture();
  ood.ood_probability = 0.8;
  expect_zero(ood);
  auto untrusted = obstacleFixture();
  untrusted.trusted_write = false;
  expect_zero(untrusted);
}

TEST(CognitiveRiskCritic, callback_admission_matches_layer_and_preserves_last_accepted)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  using Layer = bio_nav_fusion::CognitiveObstacleLayer;
  auto costmap = makeCriticTestCostmap();
  const int64_t refresh_ns = costmap->now().nanoseconds() - 10000000LL;
  const int64_t source_ns = refresh_ns - 20000000LL;
  ASSERT_TRUE(addTransform(costmap, source_ns, 0.0));
  ASSERT_TRUE(addTransform(costmap, refresh_ns, 0.0));

  auto fresh = obstacleFixture();
  auto prior = productionV310PriorFixture();
  retimeFresh(fresh, prior, source_ns);
  const Layer::Identity expected{
    fresh.reset_epoch, fresh.recurrent_session_id, fresh.map_version,
    fresh.cognitive_tile_id, fresh.tile_revision, fresh.graph_revision,
    fresh.model_id};
  Layer::AcceptanceCursor layer_cursor;

  Critic critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(fresh),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).map_version, "map");
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).source_sequence, 7U);
  Layer::recordAccepted(fresh, layer_cursor);

  auto duplicate = std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(fresh);
  EXPECT_EQ(Layer::validateMessage(
      *duplicate, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2),
    "sequence");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(critic, duplicate);
  EXPECT_NE(bio_nav_fusion::CognitiveRiskCriticTestPeer::obstacles(critic), duplicate);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);

  auto refresh = staticRevalidatedObstacleFixture();
  retimeStatic(refresh, source_ns, refresh_ns);
  EXPECT_EQ(Layer::validateMessage(
      refresh, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2), "");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(refresh),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);
  Layer::recordAccepted(refresh, layer_cursor);

  auto backward = refresh;
  const int64_t backward_ns = refresh_ns - 10000000LL;
  retimeStatic(backward, source_ns, backward_ns);
  EXPECT_EQ(Layer::validateMessage(
      backward, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2),
    "validation_regression");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(backward));
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).validation_stamp_ns,
    refresh_ns);

  auto source_regression = obstacleFixture();
  source_regression.sequence = 8;
  retimeFresh(source_regression, prior, source_ns);
  EXPECT_EQ(Layer::validateMessage(
      source_regression, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2),
    "source_regression");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(source_regression));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).source_sequence, 7U);

  auto changed = obstacleFixture();
  retimeFresh(changed, prior, refresh_ns);
  changed.sequence = 8;
  changed.map_version = "new-map";
  EXPECT_EQ(Layer::validateMessage(
      changed, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2),
    "identity");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(changed));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).map_version, "map");
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);
  const auto rejected = bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic);
  EXPECT_TRUE(rejected.valid);
  EXPECT_EQ(rejected.sequence, 8U);
  EXPECT_EQ(rejected.reason, "identity");
  EXPECT_TRUE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(critic));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusSequence(critic), 7U);
  const auto status = bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(critic);
  EXPECT_NE(status.find("accepted_source_sequence=7"), std::string::npos);
  EXPECT_NE(status.find("latest_rejected_offer_sequence=8"), std::string::npos);
  EXPECT_NE(status.find("latest_rejected_offer_reason=identity"), std::string::npos);
}

TEST(CognitiveRiskCritic, obstacle_callback_advances_before_prior_pairing_like_layer)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  using Layer = bio_nav_fusion::CognitiveObstacleLayer;
  auto costmap = makeCriticTestCostmap();
  const int64_t newer_ns = costmap->now().nanoseconds() - 10000000LL;
  const int64_t lower_ns = newer_ns - 10000000LL;
  ASSERT_TRUE(addTransform(costmap, newer_ns, 0.0));

  auto newer = obstacleFixture();
  newer.sequence = 9;
  auto newer_prior = productionV310PriorFixture();
  newer_prior.sequence = 9;
  newer_prior.local_direction_source_sequence = 9;
  retimeFresh(newer, newer_prior, newer_ns);

  Critic critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(newer));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).source_sequence, 9U);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);

  auto lower = obstacleFixture();
  lower.sequence = 8;
  auto lower_prior = productionV310PriorFixture();
  lower_prior.sequence = 8;
  lower_prior.local_direction_source_sequence = 8;
  retimeFresh(lower, lower_prior, lower_ns);
  Layer::AcceptanceCursor layer_cursor;
  Layer::recordAccepted(newer, layer_cursor);
  const Layer::Identity expected{
    newer.reset_epoch, newer.recurrent_session_id, newer.map_version,
    newer.cognitive_tile_id, newer.tile_revision, newer.graph_revision,
    newer.model_id};
  EXPECT_EQ(Layer::validateMessage(
      lower, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2),
    "sequence");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerPrior(
    critic, std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(lower_prior));
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(lower));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).source_sequence, 9U);
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::obstacles(critic)->sequence, 9U);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);

  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerPrior(
    critic, std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(newer_prior));
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);
}

TEST(CognitiveRiskCritic, reset_rebind_requires_unchanged_route_context_prior)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();
  const int64_t old_ns = costmap->now().nanoseconds() - 30000000LL;
  const int64_t reset_ns = costmap->now().nanoseconds() - 10000000LL;
  ASSERT_TRUE(addTransform(costmap, old_ns, 0.0));
  ASSERT_TRUE(addTransform(costmap, reset_ns, 0.0));

  auto old_obstacles = obstacleFixture();
  retimeFreshObstacle(old_obstacles, old_ns);

  Critic critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useAllComponents(critic);
  auto old_prior = productionV310PriorFixture();
  retimeFresh(old_obstacles, old_prior, old_ns);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(old_obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(old_prior));
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);

  auto reset_obstacles = obstacleFixture();
  reset_obstacles.reset_epoch = 4;
  reset_obstacles.recurrent_session_id = "session-reset";
  reset_obstacles.sequence = 1;
  retimeFreshObstacle(reset_obstacles, reset_ns);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(reset_obstacles));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 3U);
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason,
    "reset_route_context_missing");

  auto reset_prior = old_prior;
  reset_prior.reset_epoch = 4;
  reset_prior.recurrent_session_id = "session-reset";
  reset_prior.sequence = 99;
  reset_prior.local_direction_source_sequence = 99;
  reset_prior.stamp = stampFromNs(old_ns - 5000000000LL);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerPrior(
    critic, std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(reset_prior));
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(reset_obstacles));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 4U);
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).recurrent_session_id,
    "session-reset");
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).source_sequence, 1U);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);

  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(old_obstacles));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 4U);
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::obstacles(critic)->reset_epoch, 4U);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);

  const auto expect_reset_rejected = [&](auto mutate_prior, auto mutate_obstacle) {
      auto candidate_prior = reset_prior;
      candidate_prior.reset_epoch = 5;
      candidate_prior.recurrent_session_id = "session-rejected";
      mutate_prior(candidate_prior);
      bio_nav_fusion::CognitiveRiskCriticTestPeer::offerPrior(
        critic, std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(candidate_prior));
      auto candidate_obstacles = reset_obstacles;
      candidate_obstacles.reset_epoch = 5;
      candidate_obstacles.recurrent_session_id = "session-rejected";
      mutate_obstacle(candidate_obstacles);
      bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
        critic,
        std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(candidate_obstacles));
      EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 4U);
      EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::obstacles(critic)->reset_epoch, 4U);
    };
  const auto no_prior_change = [](auto &) {};
  const auto no_obstacle_change = [](auto &) {};
  expect_reset_rejected(
    [](auto & prior) {prior.local_direction_graph_id = "changed-route";},
    no_obstacle_change);
  expect_reset_rejected(
    [](auto & prior) {++prior.source_physical_graph_revision;},
    no_obstacle_change);
  expect_reset_rejected(
    [](auto & prior) {++prior.topology_revision;},
    no_obstacle_change);
  expect_reset_rejected(
    [](auto & prior) {prior.schema_version = "changed-schema";},
    no_obstacle_change);
  expect_reset_rejected(
    no_prior_change,
    [](auto & obstacles) {obstacles.map_version = "spoof-map";});
}

TEST(CognitiveRiskCritic, obstacle_only_reset_rebinds_without_route_context)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();
  const int64_t old_ns = costmap->now().nanoseconds() - 30000000LL;
  const int64_t reset_ns = costmap->now().nanoseconds() - 10000000LL;
  ASSERT_TRUE(addTransform(costmap, old_ns, 0.0));
  ASSERT_TRUE(addTransform(costmap, reset_ns, 0.0));

  auto old_obstacles = obstacleFixture();
  retimeFreshObstacle(old_obstacles, old_ns);
  Critic critic;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(critic, costmap);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(old_obstacles));
  ASSERT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 3U);

  auto reset_obstacles = old_obstacles;
  reset_obstacles.reset_epoch = 4;
  reset_obstacles.recurrent_session_id = "session-reset";
  reset_obstacles.sequence = 1;
  retimeFreshObstacle(reset_obstacles, reset_ns);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(reset_obstacles));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 4U);
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).recurrent_session_id,
    "session-reset");
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::cursor(critic).source_sequence, 1U);
  EXPECT_GT(scoreAt(critic, 1.0F, 0.0F), 1.0F);
  EXPECT_TRUE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(critic));
  EXPECT_NE(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(critic).find(
      "cost_delta_applied=true;obstacle_applied=true"),
    std::string::npos);

  auto replay = reset_obstacles;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(replay));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason, "sequence");

  auto old_epoch = old_obstacles;
  old_epoch.sequence = 10;
  retimeFreshObstacle(old_epoch, costmap->now().nanoseconds() - 1000000LL);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(old_epoch));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason, "identity");

  auto mismatched = reset_obstacles;
  mismatched.reset_epoch = 5;
  mismatched.recurrent_session_id = "session-mismatch";
  mismatched.map_version = "other-map";
  mismatched.sequence = 2;
  retimeFreshObstacle(mismatched, costmap->now().nanoseconds() - 1000000LL);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(mismatched));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason, "identity");
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 4U);

  auto future = reset_obstacles;
  future.reset_epoch = 5;
  future.recurrent_session_id = "session-future";
  future.sequence = 1;
  retimeFreshObstacle(future, costmap->now().nanoseconds() + 100000000LL);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(future));
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason,
    "validation_stale");

  auto stale = reset_obstacles;
  stale.reset_epoch = 5;
  stale.recurrent_session_id = "session-stale";
  stale.sequence = 1;
  retimeFreshObstacle(stale, costmap->now().nanoseconds() - 1000000000LL);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(stale));
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason,
    "validation_stale");
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch, 4U);
}

TEST(CognitiveRiskCritic, any_non_obstacle_weight_keeps_route_context_reset_gate)
{
  auto costmap = makeCriticTestCostmap();
  const int64_t old_ns = costmap->now().nanoseconds() - 30000000LL;
  const int64_t reset_ns = costmap->now().nanoseconds() - 10000000LL;
  auto old_obstacles = obstacleFixture();
  retimeFreshObstacle(old_obstacles, old_ns);
  auto reset_obstacles = old_obstacles;
  reset_obstacles.reset_epoch = 4;
  reset_obstacles.recurrent_session_id = "session-reset";
  reset_obstacles.sequence = 1;
  retimeFreshObstacle(reset_obstacles, reset_ns);

  for (const auto weights : {
      std::array<float, 3>{1.0F, 0.0F, 0.0F},
      std::array<float, 3>{0.0F, 1.0F, 0.0F},
      std::array<float, 3>{0.0F, 0.0F, 1.0F}})
  {
    bio_nav_fusion::CognitiveRiskCritic critic;
    bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(critic, costmap);
    bio_nav_fusion::CognitiveRiskCriticTestPeer::setNonObstacleWeights(
      critic, weights[0], weights[1], weights[2]);
    bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
      critic,
      std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(old_obstacles));
    bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
      critic,
      std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(reset_obstacles));
    EXPECT_EQ(
      bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason,
      "reset_route_context_missing");
    EXPECT_EQ(
      bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).reset_epoch,
      3U);
  }
}

TEST(CognitiveRiskCritic, applied_status_tracks_real_positive_component_deltas)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  auto costmap = makeCriticTestCostmap();
  const int64_t source_ns = costmap->now().nanoseconds() - 10000000LL;
  ASSERT_TRUE(addTransform(costmap, source_ns, 0.0));
  auto obstacles = obstacleFixture();
  auto prior = planningPriorFixture();
  retimeFresh(obstacles, prior, source_ns);

  const auto make_critic = [&]() {
      auto critic = std::make_unique<Critic>();
      bio_nav_fusion::CognitiveRiskCriticTestPeer::configure(*critic, costmap);
      return critic;
    };

  auto empty = obstacles;
  empty.obstacles.clear();
  auto empty_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *empty_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(empty),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_FLOAT_EQ(scoreAt(*empty_critic, 1.0F, 0.0F), 0.0F);
  EXPECT_FALSE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(*empty_critic));
  EXPECT_NE(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(*empty_critic).find(
      "zero_cost_delta;obstacle_applied=false"),
    std::string::npos);

  auto zero_weight_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useZeroWeights(*zero_weight_critic);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *zero_weight_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_FLOAT_EQ(scoreAt(*zero_weight_critic, 1.0F, 0.0F), 0.0F);
  EXPECT_FALSE(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(*zero_weight_critic));

  auto far_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *far_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_FLOAT_EQ(scoreAt(*far_critic, 1000.0F, 1000.0F), 0.0F);
  EXPECT_FALSE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(*far_critic));

  auto obstacle_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *obstacle_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_GT(scoreAt(*obstacle_critic, 1.0F, 0.0F), 0.0F);
  EXPECT_TRUE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(*obstacle_critic));
  EXPECT_NE(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(*obstacle_critic).find(
      "obstacle_applied=true"),
    std::string::npos);

  auto context_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useContextOnly(*context_critic);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *context_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_GT(scoreAt(*context_critic, 1.0F, 0.0F), 0.0F);
  const auto context_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(*context_critic);
  EXPECT_NE(context_status.find("obstacle_applied=false"), std::string::npos);
  EXPECT_NE(context_status.find("context_applied=true"), std::string::npos);
  EXPECT_NE(context_status.find("novelty_applied=true"), std::string::npos);

  auto direction_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::useDirectionOnly(*direction_critic);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *direction_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_GT(scoreAt(*direction_critic, 1.0F, 0.0F, static_cast<float>(M_PI)), 0.0F);
  const auto direction_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(*direction_critic);
  EXPECT_NE(direction_status.find("obstacle_applied=false"), std::string::npos);
  EXPECT_NE(direction_status.find("direction_applied=true"), std::string::npos);
}

TEST(CognitiveRiskCritic, nonfinite_or_negative_component_never_changes_cost)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::vector<std::array<double, 3>> trajectory{{0.0, 0.0, 0.0}};
  const std::array<double, 5> direction{};
  std::vector<Critic::ObstacleSample> obstacles{{
      0.0, 0.0, 0.1, std::numeric_limits<double>::quiet_NaN()}};
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, obstacles, direction, 0.0, 0.0, 0.0,
      1.0, 0.0, 0.0, 0.0),
    0.0);
  obstacles[0].confidence = 1.0;
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, obstacles, direction, 0.0, 0.0, 0.0,
      -1.0, 0.0, 0.0, 0.0),
    0.0);
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, obstacles, direction, 0.0,
      std::numeric_limits<double>::infinity(), 0.0,
      0.0, 0.0, 1.0, 0.0),
    0.0);
}
