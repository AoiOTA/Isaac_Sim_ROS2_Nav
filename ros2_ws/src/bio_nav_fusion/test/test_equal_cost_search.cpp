#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <thread>
#include <utility>

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
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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
    critic.last_status_ = bio_nav_interfaces::msg::RiskLayerStatus{};
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

  static void setObstacleWeight(CognitiveRiskCritic & critic, float weight)
  {
    critic.obstacle_weight_ = weight;
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

  static bio_nav_interfaces::msg::RiskLayerStatus lastStatus(
    CognitiveRiskCritic & critic)
  {
    std::lock_guard<std::mutex> lock(critic.mutex_);
    return critic.last_status_;
  }
};

class CognitiveObstacleLayerTestPeer
{
public:
  struct StaticTrackSnapshot
  {
    std::string key_track_id;
    std::string payload_track_id;
    double map_x;
    double map_y;
    double anchor_map_x;
    double anchor_map_y;
    double radius_m;
    double height_m;
    uint64_t rehit_count;
    uint64_t last_source_sequence;
    int64_t last_validation_stamp_ns;
    int64_t first_refresh_ns;
    int64_t last_refresh_ns;
    bool promoted;
    std::string reassociated_to_track_id;
  };

  static void configureActive(
    CognitiveObstacleLayer & layer,
    const bio_nav_interfaces::msg::CognitiveObstacleArray & message)
  {
    layer.mode_ = "active";
    layer.maximum_age_s_ = 0.5;
    layer.maximum_ood_probability_ = 0.2;
    layer.latest_ =
      std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(message);
    layer.latest_admission_reason_.clear();
    layer.expected_ = CognitiveObstacleLayer::Identity{
      message.reset_epoch, message.recurrent_session_id, message.map_version,
      message.cognitive_tile_id, message.tile_revision, message.graph_revision,
      message.model_id};
    layer.identity_bound_ = true;
  }

  static uint64_t observeStaticTrack(
    CognitiveObstacleLayer & layer,
    const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
    double map_x = 1.0, double map_y = 0.0)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.observeStaticTrack(
      message, message.obstacles.at(0), map_x, map_y);
  }

  static size_t staticTrackCount(CognitiveObstacleLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.static_tracks_.size();
  }

  static size_t promotedStaticTrackCount(CognitiveObstacleLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return static_cast<size_t>(std::count_if(
      layer.static_tracks_.begin(), layer.static_tracks_.end(),
             [](const auto & entry) {
               return entry.second.promoted &&
                      entry.second.reassociated_to_track_id.empty();
             }));
  }

  static std::vector<StaticTrackSnapshot> staticTracks(CognitiveObstacleLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    std::vector<StaticTrackSnapshot> tracks;
    for (const auto & [key, state] : layer.static_tracks_) {
      tracks.push_back(StaticTrackSnapshot{
          key.track_id, state.obstacle.id, state.map_x, state.map_y,
          state.anchor_map_x, state.anchor_map_y, state.radius_m, state.height_m,
          state.rehit_count, state.last_source_sequence,
          state.last_validation_stamp_ns, state.first_refresh_ns,
          state.last_refresh_ns, state.promoted,
          state.reassociated_to_track_id});
    }
    return tracks;
  }

  static std::optional<StaticTrackSnapshot> staticTrack(
    CognitiveObstacleLayer & layer, const std::string & track_id)
  {
    const auto tracks = staticTracks(layer);
    const auto track = std::find_if(
      tracks.begin(), tracks.end(), [&track_id](const auto & value) {
        return value.key_track_id == track_id;
      });
    return track == tracks.end() ? std::nullopt : std::optional(*track);
  }

  static void setClock(
    CognitiveObstacleLayer & layer, const rclcpp::Clock::SharedPtr & clock)
  {
    layer.clock_ = clock;
  }

  static void ageStaticTracks(CognitiveObstacleLayer & layer, int64_t age_ns)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    for (auto & entry : layer.static_tracks_) {
      entry.second.last_refresh_ns -= age_ns;
    }
  }

  static void ageStaticTrack(
    CognitiveObstacleLayer & layer, const std::string & track_id, int64_t age_ns)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    for (auto & [key, state] : layer.static_tracks_) {
      if (key.track_id == track_id) {
        state.last_refresh_ns -= age_ns;
      }
    }
  }

  static size_t aliasTrackCount(CognitiveObstacleLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return static_cast<size_t>(std::count_if(
      layer.static_tracks_.begin(), layer.static_tracks_.end(),
             [](const auto & entry) {
               return !entry.second.reassociated_to_track_id.empty();
             }));
  }

  static size_t appliedPromotedTrackCount(CognitiveObstacleLayer & layer)
  {
    std::lock_guard<std::mutex> lock(layer.mutex_);
    return layer.promotedStaticObstacles().size();
  }

  static double trackTtl(const CognitiveObstacleLayer & layer)
  {
    return layer.track_ttl_s_;
  }

  static void setTrackTtl(CognitiveObstacleLayer & layer, double track_ttl_s)
  {
    layer.track_ttl_s_ = track_ttl_s;
  }

  static void offer(
    CognitiveObstacleLayer & layer,
    const bio_nav_interfaces::msg::CognitiveObstacleArray & message)
  {
    layer.obstacleCallback(
      std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(message));
  }

  static void configureStatusPublisher(
    CognitiveObstacleLayer & layer,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node,
    const std::string & topic)
  {
    layer.status_publisher_ =
      node->create_publisher<bio_nav_interfaces::msg::RiskLayerStatus>(
      topic, rclcpp::QoS(10).reliable());
    layer.status_publisher_->on_activate();
  }

  static size_t statusSubscriptionCount(CognitiveObstacleLayer & layer)
  {
    return layer.status_publisher_->get_subscription_count();
  }

  static std::string applicationReason(uint32_t active_cells, uint32_t raised_cells)
  {
    return CognitiveObstacleLayer::applicationReason(active_cells, raised_cells);
  }

  static void setBeforeUpdateStatusPublishHook(
    CognitiveObstacleLayer & layer, std::function<void()> hook)
  {
    layer.before_update_status_publish_hook_ = std::move(hook);
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

class CognitiveObstacleLayerHarness : public bio_nav_fusion::CognitiveObstacleLayer
{
public:
  void bind(
    nav2_costmap_2d::LayeredCostmap & layered_costmap,
    tf2_ros::Buffer & tf_buffer, const rclcpp::Clock::SharedPtr & clock)
  {
    layered_costmap_ = &layered_costmap;
    tf_ = &tf_buffer;
    clock_ = clock;
    setDefaultValue(nav2_costmap_2d::FREE_SPACE);
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
  message.risk_model_sha256 = "risk-model-sha256";
  message.qualification_receipt_sha256 = "qualification-receipt-sha256";
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

bio_nav_interfaces::msg::CognitiveObstacle staticObstacle(
  const std::string & id, double x, double y, double radius_m = 0.06,
  uint64_t count = 3U)
{
  auto obstacle = staticRevalidatedObstacleFixture().obstacles.front();
  obstacle.id = id;
  obstacle.pose_xy_m = {x, y};
  obstacle.radius_m = radius_m;
  obstacle.count = count;
  obstacle.confidence = 1.0;
  obstacle.reliability = 1.0;
  obstacle.ood_probability = 0.0;
  return obstacle;
}

bio_nav_interfaces::msg::CognitiveObstacleArray staticBatch(
  int64_t now_ns, uint64_t sequence, int64_t validation_age_ns,
  const std::vector<bio_nav_interfaces::msg::CognitiveObstacle> & obstacles)
{
  auto message = staticRevalidatedObstacleFixture();
  message.sequence = sequence;
  message.obstacles = obstacles;
  const int64_t source_ns = now_ns - 1000000000LL;
  retimeStatic(message, source_ns, now_ns - validation_age_ns);
  for (auto & obstacle : message.obstacles) {
    obstacle.last_seen = message.header.stamp;
  }
  return message;
}

class StaticReassociationTestRig
{
public:
  StaticReassociationTestRig()
  : clock(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)),
    tf_buffer(clock), layered_costmap("map", true, false)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    layered_costmap.resizeMap(80U, 80U, 0.02, -0.8, -0.8);
    layer.bind(layered_costmap, tf_buffer, clock);
    layer.resizeMap(80U, 80U, 0.02, -0.8, -0.8);
  }

  void addTransform(
    const builtin_interfaces::msg::Time & stamp,
    double translation_x = 0.0)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.header.stamp = stamp;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = translation_x;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_buffer.setTransform(transform, "static_reassociation_test"));
  }

  void apply(
    const bio_nav_interfaces::msg::CognitiveObstacleArray & message,
    bool install_transform = true, double translation_x = 0.0)
  {
    if (install_transform) {
      addTransform(message.validation_stamp, translation_x);
    }
    layered_costmap.getCostmap()->resetMap(0U, 0U, 80U, 80U);
    bio_nav_fusion::CognitiveObstacleLayerTestPeer::configureActive(layer, message);
    layer.updateCosts(*layered_costmap.getCostmap(), 0, 0, 80, 80);
  }

  void repeatUpdate()
  {
    layered_costmap.getCostmap()->resetMap(0U, 0U, 80U, 80U);
    layer.updateCosts(*layered_costmap.getCostmap(), 0, 0, 80, 80);
  }

  uint8_t privateCost(double x, double y)
  {
    unsigned int mx = 0U;
    unsigned int my = 0U;
    EXPECT_TRUE(layer.worldToMap(x, y, mx, my));
    return layer.getCost(mx, my);
  }

  rclcpp::Clock::SharedPtr clock;
  tf2_ros::Buffer tf_buffer;
  nav2_costmap_2d::LayeredCostmap layered_costmap;
  CognitiveObstacleLayerHarness layer;
};

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

TEST(CognitiveObstacleLayer, static_depth_revalidation_accepts_independent_odom_endpoints)
{
  using bio_nav_fusion::CognitiveObstacleLayer;
  CognitiveObstacleLayer::Identity identity{
    3, "session", "map", "tile", 2, 4, "model"};
  auto message = staticRevalidatedObstacleFixture();
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      message, 11100000000LL, identity, 6, 0.5, 0.2),
    "");

  // Semantic messages and odometry commonly arrive on independent 60 Hz
  // clocks.  Each odom endpoint is tied to its own semantic endpoint, not to
  // an unrealistically exact equality between the two odom intervals.
  auto changed = message;
  changed.source_odom_stamp.nanosec = 16665668U;
  changed.validation_odom_stamp.nanosec = 16666666U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "");

  changed = message;
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
  changed.source_odom_stamp.nanosec = 100000001U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "odom_time");

  changed = message;
  changed.validation_odom_stamp.nanosec = 100000001U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "odom_time");

  changed = message;
  changed.source_odom_stamp.sec = 0;
  changed.source_odom_stamp.nanosec = 0U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "odom_time");

  changed = message;
  changed.source_odom_stamp.sec = 11;
  changed.validation_odom_stamp.sec = 10;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11100000000LL, identity, 6, 0.5, 0.2),
    "odom_time");

  changed = message;
  changed.validation_odom_stamp.sec = 11;
  changed.validation_odom_stamp.nanosec = 50000001U;
  EXPECT_EQ(
    CognitiveObstacleLayer::validateMessage(
      changed, 11000000000LL, identity, 6, 0.5, 0.2),
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

TEST(CognitiveObstacleLayer, rolling_master_origin_is_synchronized_before_cell_application)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("odom", true, false);
  constexpr double resolution = 0.05;
  constexpr double master_origin_x = 0.9499999996274724;
  constexpr double master_origin_y = -2.0000000003725287;
  constexpr double candidate_base_x = 2.1784563103032464;
  constexpr double candidate_base_y = -0.09315676066294576;
  constexpr double candidate_odom_x = 4.834160581752312;
  constexpr double candidate_odom_y = 1.070551391724286;
  layered_costmap.resizeMap(
    80U, 80U, resolution, master_origin_x, master_origin_y);
  auto * master = layered_costmap.getCostmap();

  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  // Reproduce the Pilot15 defect: same 4x4 m geometry, but the private layer
  // still has the initialization origin while the rolling master has moved.
  layer.resizeMap(80U, 80U, resolution, 0.0, 0.0);

  auto message = obstacleFixture();
  const int64_t stamp_ns = clock->now().nanoseconds() - 10000000LL;
  retimeFreshObstacle(message, stamp_ns);
  message.obstacles[0].pose_xy_m = {candidate_base_x, candidate_base_y};
  message.obstacles[0].radius_m = 0.02;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.header.stamp = message.validation_stamp;
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = candidate_odom_x - candidate_base_x;
  transform.transform.translation.y = candidate_odom_y - candidate_base_y;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "rolling_layer_test"));

  unsigned int mx = 0U;
  unsigned int my = 0U;
  EXPECT_FALSE(layer.worldToMap(candidate_odom_x, candidate_odom_y, mx, my));
  bio_nav_fusion::CognitiveObstacleLayerTestPeer::configureActive(layer, message);
  layer.updateCosts(*master, 0, 0, 80, 80);

  EXPECT_DOUBLE_EQ(layer.getOriginX(), master->getOriginX());
  EXPECT_DOUBLE_EQ(layer.getOriginY(), master->getOriginY());
  ASSERT_TRUE(layer.worldToMap(candidate_odom_x, candidate_odom_y, mx, my));
  EXPECT_EQ(layer.getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(CognitiveObstacleLayer, consecutive_origin_shifts_clear_old_cells_and_apply_new_cells)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);

  const auto apply = [&](double origin_x, double target_x, uint64_t sequence,
      int64_t stamp_ns) {
      master->updateOrigin(origin_x, -2.0);
      master->resetMap(0U, 0U, 80U, 80U);
      auto message = obstacleFixture();
      message.sequence = sequence;
      retimeFreshObstacle(message, stamp_ns);
      message.obstacles[0].pose_xy_m = {0.0, 0.0};
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "map";
      transform.header.stamp = message.validation_stamp;
      transform.child_frame_id = "base_link";
      transform.transform.translation.x = target_x;
      transform.transform.translation.y = -0.35;
      transform.transform.rotation.w = 1.0;
      ASSERT_TRUE(tf_buffer.setTransform(transform, "rolling_layer_test"));
      bio_nav_fusion::CognitiveObstacleLayerTestPeer::configureActive(layer, message);
      layer.updateCosts(*master, 0, 0, 80, 80);
      unsigned int mx = 0U;
      unsigned int my = 0U;
      ASSERT_TRUE(layer.worldToMap(target_x, -0.35, mx, my));
      EXPECT_EQ(layer.getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
      EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
    };

  const int64_t now_ns = clock->now().nanoseconds();
  apply(-2.0, -0.45, 7U, now_ns - 30000000LL);
  apply(-1.0, 1.95, 8U, now_ns - 20000000LL);
  unsigned int old_mx = 0U;
  unsigned int old_my = 0U;
  ASSERT_TRUE(layer.worldToMap(-0.45, -0.35, old_mx, old_my));
  EXPECT_EQ(layer.getCost(old_mx, old_my), nav2_costmap_2d::FREE_SPACE);

  apply(0.0, 3.45, 9U, now_ns - 10000000LL);
  EXPECT_FALSE(layer.worldToMap(-0.45, -0.35, old_mx, old_my));
  ASSERT_TRUE(layer.worldToMap(1.95, -0.35, old_mx, old_my));
  EXPECT_EQ(layer.getCost(old_mx, old_my), nav2_costmap_2d::FREE_SPACE);

  // A fixed origin takes the no-op geometry path and must keep applying cells.
  apply(0.0, 3.45, 10U, now_ns - 5000000LL);
  EXPECT_DOUBLE_EQ(layer.getOriginX(), 0.0);
  EXPECT_DOUBLE_EQ(layer.getOriginY(), -2.0);
}

TEST(CognitiveObstacleLayer, application_status_distinguishes_applied_masked_and_no_cells)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  EXPECT_EQ(Peer::applicationReason(12U, 4U), "");
  EXPECT_EQ(Peer::applicationReason(12U, 0U), "masked");
  EXPECT_EQ(Peer::applicationReason(0U, 0U), "no_costmap_cells");
}

TEST(CognitiveObstacleLayer, status_identity_and_age_follow_published_message)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  using Status = bio_nav_interfaces::msg::RiskLayerStatus;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);

  auto publisher_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "cognitive_obstacle_status_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>(
    "cognitive_obstacle_status_subscriber");
  const std::string topic = "/test/cognitive_obstacle_layer/status_identity";
  std::vector<Status> statuses;
  auto subscription = subscriber_node->create_subscription<Status>(
    topic, rclcpp::QoS(10).reliable(),
    [&statuses](const Status::SharedPtr status) {statuses.push_back(*status);});
  (void)subscription;
  Peer::configureStatusPublisher(layer, publisher_node, topic);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node->get_node_base_interface());
  executor.add_node(subscriber_node);
  for (int attempt = 0;
    attempt < 100 && Peer::statusSubscriptionCount(layer) == 0U; ++attempt)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(Peer::statusSubscriptionCount(layer), 0U);
  const auto wait_for_status_count = [&](size_t expected) {
      for (int attempt = 0; attempt < 100 && statuses.size() < expected; ++attempt) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return statuses.size() >= expected;
    };
  const auto expect_status_age = [](const Status & status,
      const bio_nav_interfaces::msg::CognitiveObstacleArray & message) {
      EXPECT_NEAR(
        rclcpp::Time(status.stamp).nanoseconds() -
        static_cast<int64_t>(std::llround(status.message_age_ms * 1.0e6)),
        rclcpp::Time(message.validation_stamp).nanoseconds(), 1000000LL);
    };

  const int64_t source_ns = clock->now().nanoseconds() - 30000000LL;
  auto accepted = obstacleFixture();
  retimeFreshObstacle(accepted, source_ns);
  Peer::offer(layer, accepted);
  ASSERT_TRUE(wait_for_status_count(1U));
  EXPECT_EQ(statuses.back().source_sequence, accepted.sequence);
  EXPECT_EQ(statuses.back().risk_model_sha256, accepted.risk_model_sha256);
  EXPECT_EQ(
    statuses.back().qualification_receipt_sha256,
    accepted.qualification_receipt_sha256);
  expect_status_age(statuses.back(), accepted);

  auto refresh = staticRevalidatedObstacleFixture();
  retimeStatic(refresh, source_ns, clock->now().nanoseconds() - 10000000LL);
  Peer::offer(layer, refresh);
  ASSERT_TRUE(wait_for_status_count(2U));
  EXPECT_EQ(statuses.back().source_sequence, accepted.sequence);
  EXPECT_EQ(statuses.back().risk_model_sha256, accepted.risk_model_sha256);
  EXPECT_EQ(
    statuses.back().qualification_receipt_sha256,
    accepted.qualification_receipt_sha256);
  expect_status_age(statuses.back(), refresh);

  auto replay = refresh;
  replay.risk_model_sha256 = "rejected-risk-model-sha256";
  replay.qualification_receipt_sha256 = "rejected-qualification-receipt-sha256";
  Peer::offer(layer, replay);
  ASSERT_TRUE(wait_for_status_count(3U));
  EXPECT_TRUE(statuses.back().rejected);
  EXPECT_EQ(statuses.back().source_sequence, replay.sequence);
  EXPECT_EQ(statuses.back().risk_model_sha256, replay.risk_model_sha256);
  EXPECT_EQ(
    statuses.back().qualification_receipt_sha256,
    replay.qualification_receipt_sha256);
  expect_status_age(statuses.back(), replay);

  auto precise = staticRevalidatedObstacleFixture();
  precise.sequence += 1U;
  const int64_t precise_validation_ns = clock->now().nanoseconds() - 10000000LL;
  const int64_t precise_source_ns = precise_validation_ns - 1234567890LL;
  retimeStatic(precise, precise_source_ns, precise_validation_ns);
  Peer::offer(layer, precise);
  ASSERT_TRUE(wait_for_status_count(4U));
  const std::string key = ";source_age_ms=";
  const auto start = statuses.back().fallback_reason.find(key);
  ASSERT_NE(start, std::string::npos);
  const auto value_start = start + key.size();
  const auto value_end = statuses.back().fallback_reason.find(';', value_start);
  ASSERT_NE(value_end, std::string::npos);
  const double encoded_ms = std::stod(
    statuses.back().fallback_reason.substr(value_start, value_end - value_start));
  EXPECT_EQ(static_cast<int64_t>(std::llround(encoded_ms * 1000000.0)), 1234567890LL);
}

TEST(CognitiveObstacleLayer, healthy_empty_update_remains_offered_but_outside_candidate_rejects)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  using Status = bio_nav_interfaces::msg::RiskLayerStatus;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);

  auto publisher_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "cognitive_obstacle_empty_status_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>(
    "cognitive_obstacle_empty_status_subscriber");
  const std::string topic = "/test/cognitive_obstacle_layer/empty_status";
  std::vector<Status> statuses;
  auto subscription = subscriber_node->create_subscription<Status>(
    topic, rclcpp::QoS(10).reliable(),
    [&statuses](const Status::SharedPtr status) {statuses.push_back(*status);});
  (void)subscription;
  Peer::configureStatusPublisher(layer, publisher_node, topic);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node->get_node_base_interface());
  executor.add_node(subscriber_node);
  const auto wait_for_status_count = [&](size_t expected) {
      for (int attempt = 0; attempt < 100 && statuses.size() < expected; ++attempt) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return statuses.size() >= expected;
    };
  for (int attempt = 0;
    attempt < 100 && Peer::statusSubscriptionCount(layer) == 0U; ++attempt)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(Peer::statusSubscriptionCount(layer), 0U);

  const int64_t now_ns = clock->now().nanoseconds();
  auto tracked = staticRevalidatedObstacleFixture();
  retimeStatic(tracked, now_ns - 1000000000LL, now_ns - 30000000LL);
  Peer::configureActive(layer, tracked);
  EXPECT_EQ(Peer::observeStaticTrack(layer, tracked), 3U);
  ASSERT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

  auto empty = tracked;
  empty.sequence += 1U;
  retimeStatic(empty, now_ns - 1000000000LL, now_ns - 20000000LL);
  empty.obstacles.clear();
  Peer::offer(layer, empty);
  EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
  ASSERT_TRUE(wait_for_status_count(1U));
  EXPECT_TRUE(statuses.back().offered);
  EXPECT_FALSE(statuses.back().applied);
  EXPECT_FALSE(statuses.back().rejected);
  EXPECT_NE(
    statuses.back().fallback_reason.find("rejection_reason=offered"),
    std::string::npos);
  master->resetMap(0U, 0U, 80U, 80U);
  layer.updateCosts(*master, 0, 0, 80, 80);
  ASSERT_TRUE(wait_for_status_count(2U));
  const auto & empty_status = statuses.back();
  EXPECT_TRUE(empty_status.offered);
  EXPECT_FALSE(empty_status.applied);
  EXPECT_FALSE(empty_status.rejected);
  EXPECT_NE(
    empty_status.fallback_reason.find("rejection_reason=offered"),
    std::string::npos);
  EXPECT_EQ(empty_status.active_cell_count, 0U);
  EXPECT_EQ(empty_status.raised_cell_count, 0U);
  EXPECT_EQ(empty_status.masked_by_existing_cost_count, 0U);
  EXPECT_EQ(empty_status.maximum_cost, 0U);
  EXPECT_EQ(empty_status.maximum_cost_increase, 0U);
  EXPECT_EQ(empty_status.source_sequence, empty.sequence);
  EXPECT_EQ(empty_status.reset_epoch, empty.reset_epoch);
  EXPECT_EQ(empty_status.map_version, empty.map_version);
  EXPECT_EQ(empty_status.risk_model_sha256, empty.risk_model_sha256);
  EXPECT_EQ(
    empty_status.qualification_receipt_sha256,
    empty.qualification_receipt_sha256);
  EXPECT_NEAR(
    rclcpp::Time(empty_status.stamp).nanoseconds() -
    static_cast<int64_t>(std::llround(empty_status.message_age_ms * 1.0e6)),
    rclcpp::Time(empty.validation_stamp).nanoseconds(), 1000000LL);

  auto outside = obstacleFixture();
  outside.sequence = empty.sequence + 1U;
  retimeFreshObstacle(outside, now_ns - 10000000LL);
  outside.obstacles[0].pose_xy_m = {100.0, 100.0};
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = outside.validation_stamp;
  transform.child_frame_id = outside.header.frame_id;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "healthy_empty_status_test"));
  Peer::offer(layer, outside);
  ASSERT_TRUE(wait_for_status_count(3U));
  master->resetMap(0U, 0U, 80U, 80U);
  layer.updateCosts(*master, 0, 0, 80, 80);
  ASSERT_TRUE(wait_for_status_count(4U));
  const auto & outside_status = statuses.back();
  EXPECT_TRUE(outside_status.offered);
  EXPECT_FALSE(outside_status.applied);
  EXPECT_TRUE(outside_status.rejected);
  EXPECT_NE(
    outside_status.fallback_reason.find("rejection_reason=no_costmap_cells"),
    std::string::npos);
  EXPECT_EQ(outside_status.active_cell_count, 0U);
  EXPECT_EQ(outside_status.raised_cell_count, 0U);
}

TEST(CognitiveObstacleLayer, rejected_empty_snapshot_cannot_retract_static_track)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  const int64_t now_ns = clock->now().nanoseconds();

  for (size_t rejection = 0U; rejection < 3U; ++rejection) {
    bio_nav_fusion::CognitiveObstacleLayer layer;
    Peer::setClock(layer, clock);
    auto tracked = staticRevalidatedObstacleFixture();
    retimeStatic(tracked, now_ns - 1000000000LL, now_ns - 30000000LL);
    Peer::configureActive(layer, tracked);
    EXPECT_EQ(Peer::observeStaticTrack(layer, tracked), 3U);
    ASSERT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

    auto empty = tracked;
    empty.sequence += 1U;
    retimeStatic(empty, now_ns - 1000000000LL, now_ns - 20000000LL);
    empty.obstacles.clear();
    if (rejection == 0U) {
      empty.trusted_write = false;
    } else if (rejection == 1U) {
      retimeStatic(empty, now_ns - 2000000000LL, now_ns - 1000000000LL);
    } else {
      empty.recurrent_session_id = "other-session";
    }
    Peer::offer(layer, empty);
    EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 1U) << rejection;
  }
}

TEST(CognitiveObstacleLayer, superseded_update_status_cannot_publish_after_new_callback)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  using Status = bio_nav_interfaces::msg::RiskLayerStatus;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);

  auto publisher_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "cognitive_obstacle_concurrent_status_publisher");
  auto subscriber_node = std::make_shared<rclcpp::Node>(
    "cognitive_obstacle_concurrent_status_subscriber");
  const std::string topic = "/test/cognitive_obstacle_layer/concurrent_status";
  std::vector<Status> statuses;
  auto subscription = subscriber_node->create_subscription<Status>(
    topic, rclcpp::QoS(10).reliable(),
    [&statuses](const Status::SharedPtr status) {statuses.push_back(*status);});
  (void)subscription;
  Peer::configureStatusPublisher(layer, publisher_node, topic);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node->get_node_base_interface());
  executor.add_node(subscriber_node);
  const auto wait_for_status_count = [&](size_t expected) {
      for (int attempt = 0; attempt < 100 && statuses.size() < expected; ++attempt) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      return statuses.size() >= expected;
    };
  for (int attempt = 0;
    attempt < 100 && Peer::statusSubscriptionCount(layer) == 0U; ++attempt)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_GT(Peer::statusSubscriptionCount(layer), 0U);

  const int64_t now_ns = clock->now().nanoseconds();
  auto first = obstacleFixture();
  retimeFreshObstacle(first, now_ns - 20000000LL);
  first.obstacles.clear();
  Peer::configureActive(layer, first);
  Peer::offer(layer, first);
  ASSERT_TRUE(wait_for_status_count(1U));

  std::promise<void> snapshot_ready;
  auto snapshot_ready_future = snapshot_ready.get_future();
  std::promise<void> resume_update;
  auto resume_update_future = resume_update.get_future();
  Peer::setBeforeUpdateStatusPublishHook(
    layer,
    [&snapshot_ready, &resume_update_future]() {
      snapshot_ready.set_value();
      resume_update_future.wait();
    });
  std::thread update_thread([&layer, master]() {
      layer.updateCosts(*master, 0, 0, 80, 80);
    });
  snapshot_ready_future.wait();

  auto second = obstacleFixture();
  second.sequence = first.sequence + 1U;
  retimeFreshObstacle(second, now_ns - 10000000LL);
  second.obstacles.clear();
  Peer::offer(layer, second);
  resume_update.set_value();
  update_thread.join();
  Peer::setBeforeUpdateStatusPublishHook(layer, {});

  ASSERT_TRUE(wait_for_status_count(2U));
  for (int attempt = 0; attempt < 10; ++attempt) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(statuses.size(), 2U);
  EXPECT_EQ(statuses.front().source_sequence, first.sequence);
  EXPECT_EQ(statuses.back().source_sequence, second.sequence);
  EXPECT_EQ(statuses.back().recurrent_session_id, second.recurrent_session_id);
  EXPECT_EQ(statuses.back().map_version, second.map_version);
  EXPECT_FALSE(statuses.back().rejected);
}

TEST(CognitiveObstacleLayer, independent_static_rehits_promote_and_persist_until_reset)
{
  using Layer = bio_nav_fusion::CognitiveObstacleLayer;
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);

  const int64_t now_ns = clock->now().nanoseconds();
  const int64_t source_ns = now_ns - 1000000000LL;
  const auto add_identity_transform = [&](int64_t stamp_ns) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "map";
      transform.header.stamp = stampFromNs(stamp_ns);
      transform.child_frame_id = "base_link";
      transform.transform.rotation.w = 1.0;
      ASSERT_TRUE(tf_buffer.setTransform(transform, "static_latch_test"));
    };
  const auto apply = [&](const bio_nav_interfaces::msg::CognitiveObstacleArray & message) {
      master->resetMap(0U, 0U, 80U, 80U);
      Peer::configureActive(layer, message);
      layer.updateCosts(*master, 0, 0, 80, 80);
      unsigned int mx = 0U;
      unsigned int my = 0U;
      EXPECT_TRUE(layer.worldToMap(1.0, 0.0, mx, my));
      return layer.getCost(mx, my);
    };

  auto first = staticRevalidatedObstacleFixture();
  first.obstacles[0].count = 1U;
  first.obstacles[0].confidence = 1.0;
  first.obstacles[0].reliability = 1.0;
  first.obstacles[0].ood_probability = 0.0;
  retimeStatic(first, source_ns, now_ns - 30000000LL);
  add_identity_transform(rclcpp::Time(first.validation_stamp).nanoseconds());
  EXPECT_EQ(apply(first), 80U);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 0U);

  EXPECT_EQ(apply(first), 80U);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 0U);

  auto second = first;
  second.obstacles[0].count = 2U;
  retimeStatic(second, source_ns, now_ns - 20000000LL);
  add_identity_transform(rclcpp::Time(second.validation_stamp).nanoseconds());
  EXPECT_EQ(apply(second), 80U);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 0U);

  auto third = second;
  retimeStatic(third, source_ns, now_ns - 10000000LL);
  add_identity_transform(rclcpp::Time(third.validation_stamp).nanoseconds());
  EXPECT_EQ(apply(third), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

  auto empty = third;
  retimeStatic(empty, source_ns, now_ns - 5000000LL);
  empty.sequence = 8U;
  empty.obstacles.clear();
  EXPECT_EQ(apply(empty), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(apply(empty), nav2_costmap_2d::LETHAL_OBSTACLE);

  auto unhealthy_empty = empty;
  unhealthy_empty.input_healthy = false;
  EXPECT_EQ(apply(unhealthy_empty), nav2_costmap_2d::LETHAL_OBSTACLE);

  auto expired_empty = empty;
  expired_empty.header.stamp = stampFromNs(now_ns - 2000000000LL);
  expired_empty.validation_stamp = stampFromNs(now_ns - 1000000000LL);
  expired_empty.source_age = durationFromNs(1000000000LL);
  expired_empty.source_odom_stamp = expired_empty.header.stamp;
  expired_empty.validation_odom_stamp = expired_empty.validation_stamp;
  EXPECT_EQ(apply(expired_empty), nav2_costmap_2d::LETHAL_OBSTACLE);

  layer.reset();
  master->resetMap(0U, 0U, 80U, 80U);
  layer.updateCosts(*master, 0, 0, 80, 80);
  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(layer.worldToMap(1.0, 0.0, mx, my));
  EXPECT_EQ(layer.getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
}

TEST(CognitiveObstacleLayer, promoted_id_chain_reassociates_to_immutable_anchor)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();

  const auto first = staticBatch(
    now_ns, 7U, 40000000LL, {staticObstacle("A", 0.0, 0.0)});
  rig.apply(first);
  auto tracks = Peer::staticTracks(rig.layer);
  ASSERT_EQ(tracks.size(), 1U);
  ASSERT_TRUE(tracks.front().promoted);
  const auto original = tracks.front();

  auto replacement_obstacle = staticObstacle("B", 0.10, 0.0, 0.07);
  replacement_obstacle.height_m = 0.30;
  replacement_obstacle.confidence = 0.95;
  const auto replacement = staticBatch(
    now_ns, 8U, 30000000LL, {replacement_obstacle});
  rig.apply(replacement);
  tracks = Peer::staticTracks(rig.layer);
  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_EQ(tracks.front().key_track_id, "A");
  EXPECT_EQ(tracks.front().payload_track_id, "A");
  EXPECT_DOUBLE_EQ(tracks.front().map_x, 0.10);
  EXPECT_DOUBLE_EQ(tracks.front().anchor_map_x, 0.0);
  EXPECT_DOUBLE_EQ(tracks.front().radius_m, 0.07);
  EXPECT_DOUBLE_EQ(tracks.front().height_m, 0.30);
  EXPECT_EQ(tracks.front().rehit_count, original.rehit_count);
  EXPECT_EQ(tracks.front().first_refresh_ns, original.first_refresh_ns);
  EXPECT_EQ(tracks.front().last_source_sequence, replacement.sequence);
  EXPECT_EQ(
    tracks.front().last_validation_stamp_ns,
    rclcpp::Time(replacement.validation_stamp).nanoseconds());
  EXPECT_GT(tracks.front().last_refresh_ns, 0);
  EXPECT_EQ(tracks[1].key_track_id, "B");
  EXPECT_EQ(tracks[1].reassociated_to_track_id, "A");
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 1U);
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 1U);
  EXPECT_EQ(rig.privateCost(0.0, 0.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(rig.privateCost(0.10, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);

  // B's current position must not become the anchor: C is accepted at 0.19 m
  // from A's original anchor, while D at 0.21 m is a distinct track.
  const auto third = staticBatch(
    now_ns, 9U, 20000000LL, {staticObstacle("C", 0.19, 0.0, 0.13)});
  rig.apply(third);
  tracks = Peer::staticTracks(rig.layer);
  ASSERT_EQ(tracks.size(), 3U);
  EXPECT_EQ(tracks.front().key_track_id, "A");
  EXPECT_DOUBLE_EQ(tracks.front().map_x, 0.19);
  EXPECT_DOUBLE_EQ(tracks.front().anchor_map_x, 0.0);

  const auto beyond_anchor = staticBatch(
    now_ns, 10U, 10000000LL, {staticObstacle("D", 0.21, 0.0, 0.13)});
  rig.apply(beyond_anchor);
  tracks = Peer::staticTracks(rig.layer);
  ASSERT_EQ(tracks.size(), 4U);
  EXPECT_EQ(tracks[0].key_track_id, "A");
  EXPECT_EQ(tracks[3].key_track_id, "D");
}

TEST(CognitiveObstacleLayer, repeated_update_of_same_latest_keeps_alias_cursor_exact_once)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0)}));
  const auto replacement = staticBatch(
    now_ns, 8U, 10000000LL, {staticObstacle("B", 0.10, 0.0, 0.07)});
  rig.apply(replacement);

  const auto canonical_before = Peer::staticTrack(rig.layer, "A");
  const auto alias_before = Peer::staticTrack(rig.layer, "B");
  ASSERT_TRUE(canonical_before.has_value());
  ASSERT_TRUE(alias_before.has_value());
  ASSERT_EQ(alias_before->reassociated_to_track_id, "A");
  for (size_t repeat = 0; repeat < 4U; ++repeat) {
    rig.repeatUpdate();
  }
  const auto canonical_after = Peer::staticTrack(rig.layer, "A");
  const auto alias_after = Peer::staticTrack(rig.layer, "B");
  ASSERT_TRUE(canonical_after.has_value());
  ASSERT_TRUE(alias_after.has_value());
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 2U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 1U);
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 1U);
  EXPECT_EQ(canonical_after->rehit_count, canonical_before->rehit_count);
  EXPECT_EQ(canonical_after->last_refresh_ns, canonical_before->last_refresh_ns);
  EXPECT_EQ(alias_after->last_refresh_ns, alias_before->last_refresh_ns);
  EXPECT_EQ(canonical_after->last_source_sequence, replacement.sequence);
  EXPECT_EQ(alias_after->last_source_sequence, replacement.sequence);
  EXPECT_EQ(rig.privateCost(0.0, 0.0), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(rig.privateCost(0.10, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(CognitiveObstacleLayer, alias_routes_each_new_cursor_to_canonical_once)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 40000000LL, {staticObstacle("A", 0.0, 0.0, 0.08)}));
  rig.apply(staticBatch(
    now_ns, 8U, 30000000LL, {staticObstacle("B", 0.05, 0.0, 0.08)}));
  const auto initial = Peer::staticTrack(rig.layer, "A");
  ASSERT_TRUE(initial.has_value());

  auto next_obstacle = staticObstacle("B", 0.08, 0.0, 0.08);
  next_obstacle.height_m = 0.31;
  const auto next = staticBatch(now_ns, 9U, 20000000LL, {next_obstacle});
  rig.apply(next);
  const auto after_next = Peer::staticTrack(rig.layer, "A");
  ASSERT_TRUE(after_next.has_value());
  EXPECT_EQ(after_next->last_source_sequence, 9U);
  EXPECT_DOUBLE_EQ(after_next->map_x, 0.08);
  EXPECT_DOUBLE_EQ(after_next->height_m, 0.31);
  EXPECT_EQ(after_next->rehit_count, initial->rehit_count);
  rig.repeatUpdate();
  EXPECT_EQ(
    Peer::staticTrack(rig.layer, "A")->last_refresh_ns,
    after_next->last_refresh_ns);

  auto final_obstacle = staticObstacle("B", 0.10, 0.0, 0.08);
  final_obstacle.height_m = 0.32;
  const auto final = staticBatch(now_ns, 10U, 10000000LL, {final_obstacle});
  rig.apply(final);
  const auto after_final = Peer::staticTrack(rig.layer, "A");
  ASSERT_TRUE(after_final.has_value());
  EXPECT_EQ(after_final->last_source_sequence, 10U);
  EXPECT_DOUBLE_EQ(after_final->map_x, 0.10);
  EXPECT_DOUBLE_EQ(after_final->height_m, 0.32);
  EXPECT_EQ(after_final->rehit_count, initial->rehit_count);
  rig.repeatUpdate();
  EXPECT_EQ(
    Peer::staticTrack(rig.layer, "A")->last_refresh_ns,
    after_final->last_refresh_ns);
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 2U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 1U);
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 1U);
}

TEST(CognitiveObstacleLayer, observed_sibling_aliases_split_before_any_canonical_update)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  const auto run = [](bool reverse) {
      StaticReassociationTestRig rig;
      const int64_t now_ns = rig.clock->now().nanoseconds();
      rig.apply(staticBatch(
        now_ns, 7U, 40000000LL, {staticObstacle("A", 0.0, 0.0)}));
      rig.apply(staticBatch(
        now_ns, 8U, 30000000LL, {staticObstacle("B", -0.03, 0.0)}));
      auto c = staticObstacle("C", 0.03, 0.0);
      c.height_m = 0.31;
      rig.apply(staticBatch(now_ns, 9U, 20000000LL, {c}));
      EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 2U);
      const auto canonical_before = Peer::staticTrack(rig.layer, "A");
      EXPECT_TRUE(canonical_before.has_value());

      auto b_observed = staticObstacle("B", -0.15, 0.0);
      b_observed.height_m = 0.41;
      auto c_observed = staticObstacle("C", 0.15, 0.0);
      c_observed.height_m = 0.42;
      std::vector<bio_nav_interfaces::msg::CognitiveObstacle> siblings{
        b_observed, c_observed};
      if (reverse) {
        std::reverse(siblings.begin(), siblings.end());
      }
      rig.apply(staticBatch(now_ns, 10U, 10000000LL, siblings));

      const auto canonical_after = Peer::staticTrack(rig.layer, "A");
      EXPECT_TRUE(canonical_after.has_value());
      EXPECT_EQ(Peer::staticTrackCount(rig.layer), 3U);
      EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 0U);
      EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 3U);
      EXPECT_EQ(canonical_after->last_source_sequence, 9U);
      EXPECT_EQ(
        canonical_after->last_validation_stamp_ns,
        canonical_before->last_validation_stamp_ns);
      EXPECT_EQ(canonical_after->last_refresh_ns, canonical_before->last_refresh_ns);
      EXPECT_DOUBLE_EQ(canonical_after->height_m, 0.31);
      EXPECT_DOUBLE_EQ(Peer::staticTrack(rig.layer, "B")->map_x, -0.15);
      EXPECT_DOUBLE_EQ(Peer::staticTrack(rig.layer, "C")->map_x, 0.15);
      EXPECT_TRUE(Peer::staticTrack(rig.layer, "B")->reassociated_to_track_id.empty());
      EXPECT_TRUE(Peer::staticTrack(rig.layer, "C")->reassociated_to_track_id.empty());
      EXPECT_EQ(rig.privateCost(-0.15, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
      EXPECT_EQ(rig.privateCost(0.15, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
      return Peer::staticTracks(rig.layer);
    };

  const auto forward = run(false);
  const auto reverse = run(true);
  ASSERT_EQ(forward.size(), reverse.size());
  for (size_t index = 0; index < forward.size(); ++index) {
    EXPECT_EQ(forward[index].key_track_id, reverse[index].key_track_id);
    EXPECT_EQ(
      forward[index].reassociated_to_track_id,
      reverse[index].reassociated_to_track_id);
    EXPECT_DOUBLE_EQ(forward[index].map_x, reverse[index].map_x);
    EXPECT_DOUBLE_EQ(forward[index].map_y, reverse[index].map_y);
    EXPECT_EQ(forward[index].last_source_sequence, reverse[index].last_source_sequence);
  }
}

TEST(CognitiveObstacleLayer, canonical_with_sibling_aliases_splits_all_observed_tracks)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 40000000LL, {staticObstacle("A", 0.0, 0.0)}));
  rig.apply(staticBatch(
    now_ns, 8U, 30000000LL, {staticObstacle("B", -0.03, 0.0)}));
  rig.apply(staticBatch(
    now_ns, 9U, 20000000LL, {staticObstacle("C", 0.03, 0.0)}));
  ASSERT_EQ(Peer::aliasTrackCount(rig.layer), 2U);

  rig.apply(staticBatch(
    now_ns, 10U, 10000000LL,
      {staticObstacle("A", 0.0, 0.0), staticObstacle("B", -0.15, 0.0),
        staticObstacle("C", 0.15, 0.0)}));
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 3U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 0U);
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 3U);
  EXPECT_TRUE(Peer::staticTrack(rig.layer, "A")->reassociated_to_track_id.empty());
  EXPECT_TRUE(Peer::staticTrack(rig.layer, "B")->reassociated_to_track_id.empty());
  EXPECT_TRUE(Peer::staticTrack(rig.layer, "C")->reassociated_to_track_id.empty());
}

TEST(CognitiveObstacleLayer, unobserved_sibling_alias_does_not_force_split)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 40000000LL, {staticObstacle("A", 0.0, 0.0, 0.08)}));
  rig.apply(staticBatch(
    now_ns, 8U, 30000000LL, {staticObstacle("B", -0.05, 0.0, 0.08)}));
  rig.apply(staticBatch(
    now_ns, 9U, 20000000LL, {staticObstacle("C", 0.05, 0.0, 0.08)}));
  ASSERT_EQ(Peer::aliasTrackCount(rig.layer), 2U);
  const auto c_before = Peer::staticTrack(rig.layer, "C");
  ASSERT_TRUE(c_before.has_value());

  rig.apply(staticBatch(
    now_ns, 10U, 10000000LL, {staticObstacle("B", -0.10, 0.0, 0.08)}));
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 3U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 2U);
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 1U);
  EXPECT_EQ(Peer::staticTrack(rig.layer, "B")->reassociated_to_track_id, "A");
  EXPECT_EQ(Peer::staticTrack(rig.layer, "C")->reassociated_to_track_id, "A");
  EXPECT_EQ(Peer::staticTrack(rig.layer, "C")->last_source_sequence, 9U);
  EXPECT_EQ(
    Peer::staticTrack(rig.layer, "C")->last_refresh_ns,
    c_before->last_refresh_ns);
  EXPECT_EQ(Peer::staticTrack(rig.layer, "A")->last_source_sequence, 10U);
  EXPECT_DOUBLE_EQ(Peer::staticTrack(rig.layer, "A")->map_x, -0.10);
}

TEST(CognitiveObstacleLayer, canonical_and_alias_coobservation_splits_real_obstacles)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0)}));
  rig.apply(staticBatch(
    now_ns, 8U, 20000000LL, {staticObstacle("B", 0.05, 0.0)}));
  ASSERT_EQ(Peer::aliasTrackCount(rig.layer), 1U);

  rig.apply(staticBatch(
    now_ns, 9U, 10000000LL,
      {staticObstacle("A", 0.0, 0.0), staticObstacle("B", 0.15, 0.0)}));
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 2U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 0U);
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 2U);
  EXPECT_TRUE(Peer::staticTrack(rig.layer, "A")->reassociated_to_track_id.empty());
  EXPECT_TRUE(Peer::staticTrack(rig.layer, "B")->reassociated_to_track_id.empty());
  EXPECT_EQ(rig.privateCost(0.0, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(rig.privateCost(0.15, 0.0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(CognitiveObstacleLayer, alias_is_removed_with_missing_canonical_ttl_and_reset)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  Peer::setTrackTtl(rig.layer, 5.0);
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0)}));
  rig.apply(staticBatch(
    now_ns, 8U, 20000000LL, {staticObstacle("B", 0.05, 0.0)}));
  ASSERT_EQ(Peer::aliasTrackCount(rig.layer), 1U);

  Peer::ageStaticTrack(rig.layer, "A", 6000000000LL);
  auto empty = staticBatch(
    now_ns, 9U, 10000000LL, {staticObstacle("unused", 0.0, 0.0)});
  empty.obstacles.clear();
  rig.apply(empty);
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 0U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 0U);

  rig.apply(staticBatch(
    now_ns, 10U, 8000000LL, {staticObstacle("A", 0.0, 0.0)}));
  rig.apply(staticBatch(
    now_ns, 11U, 5000000LL, {staticObstacle("B", 0.05, 0.0)}));
  ASSERT_EQ(Peer::aliasTrackCount(rig.layer), 1U);
  rig.layer.reset();
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 0U);
  EXPECT_EQ(Peer::aliasTrackCount(rig.layer), 0U);
}

TEST(CognitiveObstacleLayer, aliases_never_become_reassociation_targets)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0, 0.10)}));
  rig.apply(staticBatch(
    now_ns, 8U, 20000000LL, {staticObstacle("B", 0.05, 0.0, 0.10)}));
  ASSERT_EQ(Peer::staticTrack(rig.layer, "B")->reassociated_to_track_id, "A");

  rig.apply(staticBatch(
    now_ns, 9U, 10000000LL, {staticObstacle("C", 0.24, 0.0, 0.10)}));
  ASSERT_EQ(Peer::staticTrackCount(rig.layer), 3U);
  ASSERT_EQ(Peer::aliasTrackCount(rig.layer), 1U);
  EXPECT_EQ(Peer::staticTrack(rig.layer, "B")->reassociated_to_track_id, "A");
  EXPECT_TRUE(Peer::staticTrack(rig.layer, "C")->reassociated_to_track_id.empty());
  EXPECT_EQ(Peer::appliedPromotedTrackCount(rig.layer), 2U);
}

TEST(CognitiveObstacleLayer, reassociation_is_batch_order_invariant_and_mutually_unique)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  const auto run_one_old_two_new = [](bool reverse) {
      StaticReassociationTestRig rig;
      const int64_t now_ns = rig.clock->now().nanoseconds();
      rig.apply(staticBatch(
        now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0)}));
      std::vector<bio_nav_interfaces::msg::CognitiveObstacle> new_obstacles{
        staticObstacle("B", -0.05, 0.0), staticObstacle("C", 0.05, 0.0)};
      if (reverse) {
        std::reverse(new_obstacles.begin(), new_obstacles.end());
      }
      rig.apply(staticBatch(now_ns, 8U, 10000000LL, new_obstacles));
      return Peer::staticTracks(rig.layer);
    };

  const auto forward = run_one_old_two_new(false);
  const auto reverse = run_one_old_two_new(true);
  ASSERT_EQ(forward.size(), 3U);
  ASSERT_EQ(reverse.size(), forward.size());
  for (size_t index = 0; index < forward.size(); ++index) {
    EXPECT_EQ(reverse[index].key_track_id, forward[index].key_track_id);
    EXPECT_DOUBLE_EQ(reverse[index].map_x, forward[index].map_x);
    EXPECT_DOUBLE_EQ(reverse[index].anchor_map_x, forward[index].anchor_map_x);
    EXPECT_EQ(reverse[index].rehit_count, forward[index].rehit_count);
  }

  const auto run_duplicate_id = [](bool reverse) {
      StaticReassociationTestRig rig;
      const int64_t now_ns = rig.clock->now().nanoseconds();
      std::vector<bio_nav_interfaces::msg::CognitiveObstacle> duplicates{
        staticObstacle("duplicate", 0.10, 0.0),
        staticObstacle("duplicate", -0.10, 0.0)};
      if (reverse) {
        std::reverse(duplicates.begin(), duplicates.end());
      }
      rig.apply(staticBatch(now_ns, 7U, 10000000LL, duplicates));
      return Peer::staticTracks(rig.layer);
    };
  const auto duplicate_forward = run_duplicate_id(false);
  const auto duplicate_reverse = run_duplicate_id(true);
  ASSERT_EQ(duplicate_forward.size(), 1U);
  ASSERT_EQ(duplicate_reverse.size(), 1U);
  EXPECT_DOUBLE_EQ(duplicate_forward.front().map_x, -0.10);
  EXPECT_DOUBLE_EQ(duplicate_reverse.front().map_x, -0.10);

  StaticReassociationTestRig two_old_rig;
  const int64_t now_ns = two_old_rig.clock->now().nanoseconds();
  two_old_rig.apply(staticBatch(
    now_ns, 7U, 30000000LL,
      {staticObstacle("A", -0.05, 0.0), staticObstacle("B", 0.05, 0.0)}));
  two_old_rig.apply(staticBatch(
    now_ns, 8U, 10000000LL, {staticObstacle("C", 0.0, 0.0)}));
  EXPECT_EQ(Peer::staticTrackCount(two_old_rig.layer), 3U);
}

TEST(CognitiveObstacleLayer, observed_old_and_separated_static_obstacles_do_not_merge)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0, 0.08)}));
  rig.apply(staticBatch(
    now_ns, 8U, 20000000LL,
      {staticObstacle("A", 0.0, 0.0, 0.08),
        staticObstacle("B", 0.15, 0.0, 0.08)}));
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 2U);

  StaticReassociationTestRig separated_rig;
  const int64_t separated_now_ns = separated_rig.clock->now().nanoseconds();
  separated_rig.apply(staticBatch(
    separated_now_ns, 7U, 30000000LL,
      {staticObstacle("A", -0.30, 0.0, 0.20)}));
  separated_rig.apply(staticBatch(
    separated_now_ns, 8U, 10000000LL,
      {staticObstacle("B", 0.15, 0.0, 0.20)}));
  EXPECT_EQ(Peer::staticTrackCount(separated_rig.layer), 2U);
}

TEST(CognitiveObstacleLayer, reassociation_age_and_replacement_refresh_obey_horizons)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig old_rig;
  const int64_t old_now_ns = old_rig.clock->now().nanoseconds();
  old_rig.apply(staticBatch(
    old_now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0)}));
  Peer::ageStaticTracks(old_rig.layer, 2100000000LL);
  old_rig.apply(staticBatch(
    old_now_ns, 8U, 10000000LL, {staticObstacle("B", 0.05, 0.0)}));
  EXPECT_EQ(Peer::staticTrackCount(old_rig.layer), 2U);

  StaticReassociationTestRig ttl_rig;
  Peer::setTrackTtl(ttl_rig.layer, 5.0);
  const int64_t ttl_now_ns = ttl_rig.clock->now().nanoseconds();
  ttl_rig.apply(staticBatch(
    ttl_now_ns, 7U, 30000000LL, {staticObstacle("A", 0.0, 0.0)}));
  Peer::ageStaticTracks(ttl_rig.layer, 1000000000LL);
  ttl_rig.apply(staticBatch(
    ttl_now_ns, 8U, 20000000LL, {staticObstacle("B", 0.05, 0.0)}));
  ASSERT_EQ(Peer::staticTrackCount(ttl_rig.layer), 2U);
  ASSERT_EQ(Peer::aliasTrackCount(ttl_rig.layer), 1U);
  EXPECT_EQ(Peer::staticTracks(ttl_rig.layer).front().key_track_id, "A");

  auto empty = staticBatch(
    ttl_now_ns, 9U, 10000000LL, {staticObstacle("unused", 0.0, 0.0)});
  empty.obstacles.clear();
  Peer::ageStaticTracks(ttl_rig.layer, 4000000000LL);
  ttl_rig.apply(empty);
  EXPECT_EQ(Peer::staticTrackCount(ttl_rig.layer), 2U);
  Peer::ageStaticTracks(ttl_rig.layer, 2000000000LL);
  empty.sequence = 10U;
  ttl_rig.apply(empty);
  EXPECT_EQ(Peer::staticTrackCount(ttl_rig.layer), 0U);
}

TEST(CognitiveObstacleLayer, only_independently_promoted_depth_static_ids_reassociate)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig rig;
  const int64_t now_ns = rig.clock->now().nanoseconds();
  rig.apply(staticBatch(
    now_ns, 7U, 40000000LL, {staticObstacle("A", 0.0, 0.0)}));

  rig.apply(staticBatch(
    now_ns, 8U, 30000000LL, {staticObstacle("B", 0.05, 0.0, 0.06, 1U)}));
  auto tracks = Peer::staticTracks(rig.layer);
  ASSERT_EQ(tracks.size(), 2U);
  EXPECT_TRUE(tracks[0].promoted);
  EXPECT_FALSE(tracks[1].promoted);

  auto fresh_dynamic = obstacleFixture();
  fresh_dynamic.sequence = 9U;
  fresh_dynamic.obstacles = {staticObstacle("dynamic", 0.04, 0.0)};
  fresh_dynamic.obstacles[0].motion_class =
    bio_nav_interfaces::msg::CognitiveObstacle::MOTION_DYNAMIC;
  fresh_dynamic.obstacles[0].static_confirmed = false;
  retimeFreshObstacle(fresh_dynamic, now_ns - 10000000LL);
  rig.apply(fresh_dynamic);
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 2U);

  auto wrong_validation = staticBatch(
    now_ns, 10U, 5000000LL, {staticObstacle("wrong-validation", 0.03, 0.0)});
  wrong_validation.validation_sensor_mask = 0U;
  rig.apply(wrong_validation);
  EXPECT_EQ(Peer::staticTrackCount(rig.layer), 2U);
}

TEST(CognitiveObstacleLayer, nonfinite_transforms_are_rejected_and_huge_radius_is_bounded)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  StaticReassociationTestRig nonfinite_rig;
  const int64_t now_ns = nonfinite_rig.clock->now().nanoseconds();
  auto nan_radius = staticObstacle("nan-radius", 0.0, 0.0);
  nan_radius.radius_m = std::numeric_limits<double>::quiet_NaN();
  nonfinite_rig.apply(staticBatch(
    now_ns, 6U, 30000000LL, {nan_radius}));
  EXPECT_EQ(Peer::staticTrackCount(nonfinite_rig.layer), 0U);

  auto nonfinite_target = staticObstacle(
    "overflow", std::numeric_limits<double>::max(), 0.0);
  const auto overflow_message = staticBatch(
    now_ns, 7U, 20000000LL, {nonfinite_target});
  nonfinite_rig.apply(
    overflow_message, true, std::numeric_limits<double>::max());
  EXPECT_EQ(Peer::staticTrackCount(nonfinite_rig.layer), 0U);

  StaticReassociationTestRig huge_rig;
  const int64_t huge_now_ns = huge_rig.clock->now().nanoseconds();
  auto huge = staticObstacle("huge", 0.0, 0.0);
  huge.radius_m = std::numeric_limits<double>::max();
  huge_rig.apply(staticBatch(
    huge_now_ns, 7U, 10000000LL, {huge}));
  EXPECT_EQ(Peer::staticTrackCount(huge_rig.layer), 1U);
  EXPECT_EQ(huge_rig.privateCost(-0.79, -0.79), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(huge_rig.privateCost(0.79, 0.79), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(CognitiveObstacleLayer, static_latch_key_clears_for_each_identity_rollover)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(20U, 20U, 0.1, -1.0, -1.0);
  const int64_t validation_ns = clock->now().nanoseconds() - 10000000LL;
  const int64_t source_ns = validation_ns - 1000000000LL;

  for (size_t field = 0; field < 7U; ++field) {
    CognitiveObstacleLayerHarness layer;
    layer.bind(layered_costmap, tf_buffer, clock);
    layer.resizeMap(20U, 20U, 0.1, -1.0, -1.0);
    auto message = staticRevalidatedObstacleFixture();
    retimeStatic(message, source_ns, validation_ns);
    Peer::configureActive(layer, message);
    EXPECT_EQ(Peer::observeStaticTrack(layer, message), 3U);
    ASSERT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

    auto rollover = message;
    switch (field) {
      case 0: ++rollover.reset_epoch; break;
      case 1: rollover.recurrent_session_id = "next-session"; break;
      case 2: rollover.map_version = "next-map"; break;
      case 3: rollover.cognitive_tile_id = "next-tile"; break;
      case 4: ++rollover.tile_revision; break;
      case 5: ++rollover.graph_revision; break;
      case 6: rollover.model_id = "next-model"; break;
    }
    Peer::offer(layer, rollover);
    EXPECT_EQ(Peer::staticTrackCount(layer), 0U) << "identity field " << field;
  }
}

TEST(CognitiveObstacleLayer, rejected_or_soft_candidates_never_create_a_lethal_latch)
{
  using Layer = bio_nav_fusion::CognitiveObstacleLayer;
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  const int64_t validation_ns = clock->now().nanoseconds() - 10000000LL;
  const int64_t source_ns = validation_ns - 1000000000LL;

  const auto expect_no_track = [&](bio_nav_interfaces::msg::CognitiveObstacleArray message,
      bool provide_tf) {
      tf2_ros::Buffer tf_buffer(clock);
      CognitiveObstacleLayerHarness layer;
      layer.bind(layered_costmap, tf_buffer, clock);
      layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
      retimeStatic(message, source_ns, validation_ns);
      if (provide_tf) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "map";
        transform.header.stamp = message.validation_stamp;
        transform.child_frame_id = "base_link";
        transform.transform.rotation.w = 1.0;
        EXPECT_TRUE(tf_buffer.setTransform(transform, "static_rejection_test"));
      }
      Peer::configureActive(layer, message);
      master->resetMap(0U, 0U, 80U, 80U);
      layer.updateCosts(*master, 0, 0, 80, 80);
      EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
    };

  auto untrusted = staticRevalidatedObstacleFixture();
  untrusted.trusted_write = false;
  expect_no_track(untrusted, true);
  auto ood = staticRevalidatedObstacleFixture();
  ood.ood_probability = 0.8;
  expect_no_track(ood, true);
  auto dynamic = staticRevalidatedObstacleFixture();
  dynamic.obstacles[0].motion_class =
    bio_nav_interfaces::msg::CognitiveObstacle::MOTION_DYNAMIC;
  expect_no_track(dynamic, true);
  expect_no_track(staticRevalidatedObstacleFixture(), false);

  Layer soft_layer;
  Peer::setClock(soft_layer, clock);
  auto soft = staticRevalidatedObstacleFixture();
  soft.obstacles[0].count = 1U;
  soft.obstacles[0].confidence = 0.5;
  for (uint32_t rehit = 0U; rehit < 3U; ++rehit) {
    soft.validation_stamp.nanosec += 1U;
    soft.source_age.nanosec += 1U;
    soft.validation_odom_stamp.nanosec += 1U;
    const auto effective_count = Peer::observeStaticTrack(soft_layer, soft);
    EXPECT_LT(
      Layer::obstacleCost(soft.obstacles[0], effective_count, 80, 0.02, 0.45),
      nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  EXPECT_EQ(Peer::promotedStaticTrackCount(soft_layer), 0U);
}

TEST(CognitiveObstacleLayer, static_track_ttl_defaults_to_ninety_seconds)
{
  bio_nav_fusion::CognitiveObstacleLayer layer;
  EXPECT_DOUBLE_EQ(
    bio_nav_fusion::CognitiveObstacleLayerTestPeer::trackTtl(layer), 90.0);
}

TEST(CognitiveObstacleLayer, static_track_ttl_expires_silenced_track_and_frees_cells)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  Peer::setTrackTtl(layer, 5.0);

  const int64_t now_ns = clock->now().nanoseconds();
  const int64_t source_ns = now_ns - 1000000000LL;
  auto message = staticRevalidatedObstacleFixture();
  retimeStatic(message, source_ns, now_ns - 30000000LL);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = message.validation_stamp;
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "static_ttl_test"));

  const auto apply = [&](const bio_nav_interfaces::msg::CognitiveObstacleArray & current) {
      master->resetMap(0U, 0U, 80U, 80U);
      Peer::configureActive(layer, current);
      layer.updateCosts(*master, 0, 0, 80, 80);
      unsigned int mx = 0U;
      unsigned int my = 0U;
      EXPECT_TRUE(layer.worldToMap(1.0, 0.0, mx, my));
      return layer.getCost(mx, my);
    };

  // One hard static report promotes the track and latches the cell.
  EXPECT_EQ(apply(message), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

  // Module2 stops reporting the obstacle; within the TTL the latch holds.
  auto empty = message;
  retimeStatic(empty, source_ns, now_ns - 20000000LL);
  empty.sequence = 8U;
  empty.obstacles.clear();
  EXPECT_EQ(apply(empty), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);

  // After the TTL horizon the ghost track is dropped and stops writing cells.
  Peer::ageStaticTracks(layer, 6000000000LL);
  auto later_empty = message;
  retimeStatic(later_empty, source_ns, now_ns - 10000000LL);
  later_empty.sequence = 9U;
  later_empty.obstacles.clear();
  EXPECT_EQ(apply(later_empty), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 0U);
  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(layer.worldToMap(1.0, 0.0, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
}

TEST(CognitiveObstacleLayer, static_track_ttl_survives_while_reports_continue)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  Peer::setTrackTtl(layer, 5.0);

  const int64_t now_ns = clock->now().nanoseconds();
  const int64_t source_ns = now_ns - 1000000000LL;
  const auto add_identity_transform = [&](const builtin_interfaces::msg::Time & stamp) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id = "map";
      transform.header.stamp = stamp;
      transform.child_frame_id = "base_link";
      transform.transform.rotation.w = 1.0;
      ASSERT_TRUE(tf_buffer.setTransform(transform, "static_ttl_test"));
    };
  const auto apply = [&](const bio_nav_interfaces::msg::CognitiveObstacleArray & current) {
      master->resetMap(0U, 0U, 80U, 80U);
      Peer::configureActive(layer, current);
      layer.updateCosts(*master, 0, 0, 80, 80);
      unsigned int mx = 0U;
      unsigned int my = 0U;
      EXPECT_TRUE(layer.worldToMap(1.0, 0.0, mx, my));
      return layer.getCost(mx, my);
    };
  const auto emptyAfter = [&](const bio_nav_interfaces::msg::CognitiveObstacleArray & base,
      uint64_t sequence, int64_t validation_ns) {
      auto empty = base;
      retimeStatic(empty, source_ns, validation_ns);
      empty.sequence = sequence;
      empty.obstacles.clear();
      return empty;
    };

  auto message = staticRevalidatedObstacleFixture();
  retimeStatic(message, source_ns, now_ns - 30000000LL);
  add_identity_transform(message.validation_stamp);
  EXPECT_EQ(apply(message), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

  // Silence for 4 s (< 5 s TTL): the latch still holds.
  Peer::ageStaticTracks(layer, 4000000000LL);
  EXPECT_EQ(
    apply(emptyAfter(message, 8U, now_ns - 20000000LL)),
    nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);

  // A fresh independent report refreshes the track and restarts the horizon.
  auto refresh = message;
  refresh.sequence = 9U;
  retimeStatic(refresh, source_ns, now_ns - 15000000LL);
  add_identity_transform(refresh.validation_stamp);
  EXPECT_EQ(apply(refresh), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);

  // 4 s after the refresh the track survives; 6 s after, it is gone.
  Peer::ageStaticTracks(layer, 4000000000LL);
  EXPECT_EQ(
    apply(emptyAfter(message, 10U, now_ns - 10000000LL)),
    nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);
  Peer::ageStaticTracks(layer, 2000000000LL);
  EXPECT_EQ(
    apply(emptyAfter(message, 11U, now_ns - 5000000LL)),
    nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
}

TEST(CognitiveObstacleLayer, unpromoted_static_track_expires_with_ttl)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  Peer::setTrackTtl(layer, 5.0);

  const int64_t now_ns = clock->now().nanoseconds();
  const int64_t source_ns = now_ns - 1000000000LL;
  auto message = staticRevalidatedObstacleFixture();
  message.obstacles[0].count = 1U;
  message.obstacles[0].confidence = 0.5;
  retimeStatic(message, source_ns, now_ns - 30000000LL);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = message.validation_stamp;
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "static_ttl_test"));

  // A soft candidate creates a track but never promotes.
  Peer::configureActive(layer, message);
  layer.updateCosts(*master, 0, 0, 80, 80);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);
  EXPECT_EQ(Peer::promotedStaticTrackCount(layer), 0U);

  // The rehit state must not live forever without promotion either.
  Peer::ageStaticTracks(layer, 6000000000LL);
  auto empty = message;
  retimeStatic(empty, source_ns, now_ns - 10000000LL);
  empty.sequence = 8U;
  empty.obstacles.clear();
  Peer::configureActive(layer, empty);
  master->resetMap(0U, 0U, 80U, 80U);
  layer.updateCosts(*master, 0, 0, 80, 80);
  EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(layer.worldToMap(1.0, 0.0, mx, my));
  EXPECT_EQ(layer.getCost(mx, my), nav2_costmap_2d::FREE_SPACE);
}

TEST(CognitiveObstacleLayer, update_bounds_stops_touching_after_static_track_ttl)
{
  using Peer = bio_nav_fusion::CognitiveObstacleLayerTestPeer;
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer tf_buffer(clock);
  nav2_costmap_2d::LayeredCostmap layered_costmap("map", true, false);
  layered_costmap.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  auto * master = layered_costmap.getCostmap();
  CognitiveObstacleLayerHarness layer;
  layer.bind(layered_costmap, tf_buffer, clock);
  layer.resizeMap(80U, 80U, 0.05, -2.0, -2.0);
  Peer::setTrackTtl(layer, 5.0);

  const int64_t now_ns = clock->now().nanoseconds();
  const int64_t source_ns = now_ns - 1000000000LL;
  auto message = staticRevalidatedObstacleFixture();
  retimeStatic(message, source_ns, now_ns - 30000000LL);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = message.validation_stamp;
  transform.child_frame_id = "base_link";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_buffer.setTransform(transform, "static_ttl_test"));
  Peer::configureActive(layer, message);
  layer.updateCosts(*master, 0, 0, 80, 80);
  ASSERT_EQ(Peer::promotedStaticTrackCount(layer), 1U);

  // A stale offer makes updateCosts drop latest_, leaving the live track as
  // the only reason updateBounds expands the rolling window.
  auto stale = message;
  retimeStatic(stale, source_ns - 9000000000LL, now_ns - 10000000000LL);
  stale.obstacles.clear();
  Peer::configureActive(layer, stale);
  layer.updateCosts(*master, 0, 0, 80, 80);

  double min_x = -1.0;
  double min_y = -1.0;
  double max_x = 1.0;
  double max_y = 1.0;
  layer.updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, -1000.0);
  EXPECT_DOUBLE_EQ(max_x, 1000.0);
  EXPECT_EQ(Peer::staticTrackCount(layer), 1U);

  // Once the track outlives the TTL, updateBounds prunes it and no longer
  // touches the window for it.
  Peer::ageStaticTracks(layer, 6000000000LL);
  min_x = -1.0;
  min_y = -1.0;
  max_x = 1.0;
  max_y = 1.0;
  layer.updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, -1.0);
  EXPECT_DOUBLE_EQ(max_x, 1.0);
  EXPECT_EQ(Peer::staticTrackCount(layer), 0U);
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

TEST(CognitiveRiskCritic, duplicate_and_overlapping_candidates_are_count_invariant)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::vector<std::array<double, 3>> trajectory{
    {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}};
  const std::array<double, 5> no_direction{};
  const Critic::ObstacleSample obstacle{0.25, 0.0, 0.5, 0.8};
  const std::vector<Critic::ObstacleSample> single{obstacle};
  const std::vector<Critic::ObstacleSample> duplicates(16U, obstacle);
  const auto single_score = Critic::trajectoryScore(
    trajectory, single, no_direction, 0.0, 0.0, 0.0,
    4.0, 0.0, 0.0, 0.0);
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, duplicates, no_direction, 0.0, 0.0, 0.0,
      4.0, 0.0, 0.0, 0.0),
    single_score);

  const Critic::ObstacleSample weaker_overlap{0.25, 0.0, 0.5, 0.3};
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, {weaker_overlap, obstacle}, no_direction, 0.0, 0.0, 0.0,
      4.0, 0.0, 0.0, 0.0),
    single_score);
}

TEST(CognitiveRiskCritic, repeated_identical_poses_do_not_increase_obstacle_score)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::array<double, 5> no_direction{};
  const Critic::ObstacleSample obstacle{0.0, 0.0, 0.2, 0.7};
  const std::vector<std::array<double, 3>> twenty_poses(
    20U, std::array<double, 3>{0.0, 0.0, 0.0});
  const std::vector<std::array<double, 3>> forty_poses(
    40U, std::array<double, 3>{0.0, 0.0, 0.0});
  const auto score_twenty = Critic::trajectoryScore(
    twenty_poses, {obstacle}, no_direction, 0.0, 0.0, 0.0,
    4.0, 0.0, 0.0, 0.0);
  const auto score_forty = Critic::trajectoryScore(
    forty_poses, {obstacle}, no_direction, 0.0, 0.0, 0.0,
    4.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(score_twenty, score_forty, 1.0e-12);
  EXPECT_NEAR(score_twenty, 5.6, 1.0e-12);
}

TEST(CognitiveRiskCritic, obstacle_score_is_stable_across_horizon_discretization)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::array<double, 5> no_direction{};
  const Critic::ObstacleSample obstacle{1.0, 0.0, 0.2, 1.0};
  const auto sample_path = [](std::size_t samples, double y) {
      std::vector<std::array<double, 3>> trajectory;
      trajectory.reserve(samples);
      for (std::size_t index = 0; index < samples; ++index) {
        const double x = (static_cast<double>(index) + 0.5) * 2.0 /
          static_cast<double>(samples);
        trajectory.push_back({x, y, 0.0});
      }
      return trajectory;
    };
  const auto score_twenty = Critic::trajectoryScore(
    sample_path(20U, 0.5), {obstacle}, no_direction, 0.0, 0.0, 0.0,
    4.0, 0.0, 0.0, 0.0);
  const auto score_forty = Critic::trajectoryScore(
    sample_path(40U, 0.5), {obstacle}, no_direction, 0.0, 0.0, 0.0,
    4.0, 0.0, 0.0, 0.0);
  EXPECT_NEAR(score_twenty, score_forty, 0.01);
}

TEST(CognitiveRiskCritic, far_near_and_collision_obstacle_scores_are_ordered)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::array<double, 5> no_direction{};
  const Critic::ObstacleSample obstacle{0.0, 0.0, 0.2, 1.0};
  const auto score_at = [&](double y) {
      const std::vector<std::array<double, 3>> trajectory{
        {0.0, y, 0.0}, {0.0, y, 0.0}};
      return Critic::trajectoryScore(
        trajectory, {obstacle}, no_direction, 0.0, 0.0, 0.0,
        4.0, 0.0, 0.0, 0.0);
    };
  const auto far_score = score_at(2.0);
  const auto near_score = score_at(0.5);
  const auto collision_score = score_at(0.0);
  EXPECT_LT(far_score, near_score);
  EXPECT_LT(near_score, collision_score);
  EXPECT_DOUBLE_EQ(collision_score, 8.0);
}

TEST(CognitiveRiskCritic, nonoverlapping_candidates_apply_at_their_own_time_steps)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::vector<std::array<double, 3>> trajectory{
    {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const std::array<double, 5> no_direction{};
  const Critic::ObstacleSample first{0.0, 0.0, 0.2, 1.0};
  const Critic::ObstacleSample second{10.0, 0.0, 0.2, 1.0};
  const auto first_only = Critic::trajectoryScore(
    trajectory, {first}, no_direction, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0);
  const auto second_only = Critic::trajectoryScore(
    trajectory, {second}, no_direction, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0);
  const auto both = Critic::trajectoryScore(
    trajectory, {first, second}, no_direction, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0);
  EXPECT_GT(both, first_only);
  EXPECT_GT(both, second_only);
  EXPECT_DOUBLE_EQ(both, 2.0);
}

TEST(CognitiveRiskCritic, max_per_step_mean_score_is_finite_and_respects_obstacle_weight)
{
  using Critic = bio_nav_fusion::CognitiveRiskCritic;
  const std::vector<std::array<double, 3>> trajectory{{0.0, 0.0, 0.0}};
  const std::array<double, 5> no_direction{};
  const Critic::ObstacleSample finite{0.0, 0.0, 0.2, 0.75};
  const Critic::ObstacleSample nonfinite{
    0.0, 0.0, 0.2, std::numeric_limits<double>::quiet_NaN()};
  const auto unit_weight = Critic::trajectoryScore(
    trajectory, {nonfinite, finite}, no_direction, 0.0, 0.0, 0.0,
    1.0, 0.0, 0.0, 0.0);
  EXPECT_TRUE(std::isfinite(unit_weight));
  EXPECT_DOUBLE_EQ(unit_weight, 1.5);
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, {finite}, no_direction, 0.0, 0.0, 0.0,
      3.0, 0.0, 0.0, 0.0),
    3.0 * unit_weight);
  EXPECT_DOUBLE_EQ(
    Critic::trajectoryScore(
      trajectory, {finite}, no_direction, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0),
    0.0);
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
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::appliedStatus(
      "not_required_obstacle_only", "", ""),
    "cost_delta_applied=true;obstacle_applied=true;active_effect_scope=obstacle_only"
    ";prior_required=false");
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
  const auto accepted_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(critic);
  EXPECT_EQ(accepted_status.source_sequence, fresh.sequence);
  EXPECT_EQ(accepted_status.reset_epoch, fresh.reset_epoch);
  EXPECT_EQ(accepted_status.recurrent_session_id, fresh.recurrent_session_id);
  EXPECT_EQ(accepted_status.map_version, fresh.map_version);
  EXPECT_EQ(accepted_status.risk_model_sha256, fresh.risk_model_sha256);
  EXPECT_EQ(
    accepted_status.qualification_receipt_sha256,
    fresh.qualification_receipt_sha256);
  EXPECT_GT(accepted_status.message_age_ms, 0.0F);
  EXPECT_NEAR(
    rclcpp::Time(accepted_status.stamp).nanoseconds() -
    static_cast<int64_t>(std::llround(accepted_status.message_age_ms * 1.0e6)),
    rclcpp::Time(fresh.validation_stamp).nanoseconds(), 1000000LL);
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
  const auto refresh_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(critic);
  EXPECT_EQ(refresh_status.source_sequence, fresh.sequence);
  EXPECT_EQ(refresh_status.risk_model_sha256, fresh.risk_model_sha256);
  EXPECT_EQ(
    refresh_status.qualification_receipt_sha256,
    fresh.qualification_receipt_sha256);
  EXPECT_NEAR(
    rclcpp::Time(refresh_status.stamp).nanoseconds() -
    static_cast<int64_t>(std::llround(refresh_status.message_age_ms * 1.0e6)),
    rclcpp::Time(refresh.validation_stamp).nanoseconds(), 1000000LL);
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
  changed.risk_model_sha256 = "rejected-risk-model-sha256";
  changed.qualification_receipt_sha256 = "rejected-qualification-receipt-sha256";
  EXPECT_EQ(Layer::validateMessage(
      changed, costmap->now().nanoseconds(), expected, layer_cursor, 0.5, 0.2),
    "identity");
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(changed));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::identity(critic).map_version, "map");
  const auto rejection_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(critic);
  EXPECT_FALSE(rejection_status.applied);
  EXPECT_TRUE(rejection_status.rejected);
  EXPECT_EQ(rejection_status.source_sequence, fresh.sequence);
  EXPECT_EQ(rejection_status.reset_epoch, fresh.reset_epoch);
  EXPECT_EQ(rejection_status.recurrent_session_id, fresh.recurrent_session_id);
  EXPECT_EQ(rejection_status.map_version, fresh.map_version);
  EXPECT_EQ(rejection_status.risk_model_sha256, fresh.risk_model_sha256);
  EXPECT_EQ(
    rejection_status.qualification_receipt_sha256,
    fresh.qualification_receipt_sha256);
  EXPECT_NEAR(
    rclcpp::Time(rejection_status.stamp).nanoseconds() -
    static_cast<int64_t>(std::llround(rejection_status.message_age_ms * 1.0e6)),
    rclcpp::Time(refresh.validation_stamp).nanoseconds(), 1000000LL);
  EXPECT_NE(
    rejection_status.fallback_reason.find("offer_reset_epoch=3"),
    std::string::npos);
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
  EXPECT_FALSE(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).valid);
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
  const auto expect_accepted_status_identity = [&]() {
      const auto status = bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(critic);
      EXPECT_EQ(status.source_sequence, reset_obstacles.sequence);
      EXPECT_EQ(status.reset_epoch, reset_obstacles.reset_epoch);
      EXPECT_EQ(status.recurrent_session_id, reset_obstacles.recurrent_session_id);
      EXPECT_EQ(status.map_version, reset_obstacles.map_version);
    };
  expect_accepted_status_identity();
  EXPECT_TRUE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(critic));
  EXPECT_NE(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusReason(critic).find(
      "cost_delta_applied=true;obstacle_applied=true"),
    std::string::npos);

  auto replay = reset_obstacles;
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(replay));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason, "sequence");
  expect_accepted_status_identity();

  auto old_epoch = old_obstacles;
  old_epoch.sequence = 10;
  retimeFreshObstacle(old_epoch, costmap->now().nanoseconds() - 1000000LL);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::offerObstacle(
    critic, std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(old_epoch));
  EXPECT_EQ(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastRejected(critic).reason, "identity");
  expect_accepted_status_identity();

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
  expect_accepted_status_identity();

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
  expect_accepted_status_identity();

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
  expect_accepted_status_identity();
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
  const auto obstacle_delta = scoreAt(*obstacle_critic, 1.0F, 0.0F);
  EXPECT_GT(obstacle_delta, 0.0F);
  EXPECT_TRUE(bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatusApplied(*obstacle_critic));
  const auto obstacle_status =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(*obstacle_critic);
  EXPECT_EQ(obstacle_status.maximum_cost_increase, 1U);
  EXPECT_NE(obstacle_status.fallback_reason.find("obstacle_applied=true"), std::string::npos);
  EXPECT_NE(obstacle_status.fallback_reason.find("obstacle_count=1"), std::string::npos);
  EXPECT_NE(
    obstacle_status.fallback_reason.find("aggregation=max_per_step_mean_horizon"),
    std::string::npos);
  const std::string maximum_key = "maximum_obstacle_cost_delta=";
  const auto maximum_start = obstacle_status.fallback_reason.find(maximum_key);
  ASSERT_NE(maximum_start, std::string::npos);
  const auto maximum_value_start = maximum_start + maximum_key.size();
  const auto maximum_end = obstacle_status.fallback_reason.find(';', maximum_value_start);
  ASSERT_NE(maximum_end, std::string::npos);
  const auto reported_maximum = std::stod(
    obstacle_status.fallback_reason.substr(
      maximum_value_start, maximum_end - maximum_value_start));
  EXPECT_TRUE(std::isfinite(reported_maximum));
  EXPECT_FLOAT_EQ(static_cast<float>(reported_maximum), obstacle_delta);

  auto duplicate_obstacles = obstacles;
  duplicate_obstacles.obstacles.push_back(duplicate_obstacles.obstacles.front());
  duplicate_obstacles.obstacles.back().id = "object-overlap";
  auto duplicate_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *duplicate_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(duplicate_obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_FLOAT_EQ(scoreAt(*duplicate_critic, 1.0F, 0.0F), obstacle_delta);
  const auto duplicate_status_reason =
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(
      *duplicate_critic).fallback_reason;
  EXPECT_NE(
    duplicate_status_reason.find("obstacle_count=2"),
    std::string::npos) << duplicate_status_reason;

  auto clipped_critic = make_critic();
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setObstacleWeight(*clipped_critic, 1000.0F);
  bio_nav_fusion::CognitiveRiskCriticTestPeer::setInputs(
    *clipped_critic,
    std::make_shared<bio_nav_interfaces::msg::CognitiveObstacleArray>(obstacles),
    std::make_shared<bio_nav_interfaces::msg::PlanningPrior>(prior));
  EXPECT_GT(scoreAt(*clipped_critic, 1.0F, 0.0F), 255.0F);
  EXPECT_EQ(
    bio_nav_fusion::CognitiveRiskCriticTestPeer::lastStatus(
      *clipped_critic).maximum_cost_increase,
    std::numeric_limits<uint8_t>::max());

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
