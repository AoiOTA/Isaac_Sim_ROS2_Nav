#ifndef BIO_NAV_FUSION__COGNITIVE_RISK_CRITIC_HPP_
#define BIO_NAV_FUSION__COGNITIVE_RISK_CRITIC_HPP_

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bio_nav_fusion/cognitive_obstacle_layer.hpp"
#include "bio_nav_interfaces/msg/cognitive_obstacle_array.hpp"
#include "bio_nav_interfaces/msg/planning_prior.hpp"
#include "bio_nav_interfaces/msg/risk_layer_status.hpp"
#include "nav2_mppi_controller/critic_function.hpp"

namespace bio_nav_fusion
{

class CognitiveRiskCritic : public mppi::critics::CriticFunction
{
public:
  struct ObstacleSample
  {
    double x{0.0};
    double y{0.0};
    double radius{0.0};
    double confidence{0.0};
  };

  struct RouteContext
  {
    std::string planning_schema;
    std::string direction_schema;
    std::string route_graph_id;
    std::string physical_graph_id;
    uint64_t physical_graph_revision{0};
    uint64_t topology_revision{0};
  };

  struct RejectedOffer
  {
    uint64_t sequence{0};
    uint32_t reset_epoch{0};
    std::string recurrent_session_id;
    std::string reason;
    bool valid{false};
  };

  void initialize() override;
  void score(mppi::CriticData & data) override;

  static double trajectoryScore(
    const std::vector<std::array<double, 3>> & trajectory,
    const std::vector<ObstacleSample> & obstacles,
    const std::array<double, 5> & direction_weights,
    double robot_yaw, double novelty, double uncertainty, double obstacle_weight,
    double direction_weight, double novelty_weight,
    double uncertainty_weight);
  static std::string validateInputs(
    const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
    const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
    double maximum_age_s, double maximum_ood_probability);
  static std::string validateInputs(
    const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
    const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
    const CognitiveObstacleLayer::Identity & expected,
    const CognitiveObstacleLayer::AcceptanceCursor & accepted,
    bool enforce_identity, double maximum_age_s,
    double maximum_ood_probability);
  static std::string validatePriorComponents(
    const bio_nav_interfaces::msg::CognitiveObstacleArray * obstacles,
    const bio_nav_interfaces::msg::PlanningPrior * prior, int64_t now_ns,
    double maximum_age_s, double maximum_ood_probability);
  static std::string validateDirectionPrior(
    const bio_nav_interfaces::msg::PlanningPrior & prior,
    double prior_age_s = 0.0);

private:
  friend class CognitiveRiskCriticTestPeer;

  void obstacleCallback(
    const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message);
  void priorCallback(
    const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message);
  void publishStatus(
    const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr & accepted_obstacles,
    bool applied, const std::string & reason);
  static std::string appliedStatus(
    const std::string & prior_reason, const std::string & context_reason,
    const std::string & direction_reason, bool obstacle_applied,
    bool novelty_applied, bool uncertainty_applied, bool direction_applied);
  static std::string validateRouteContext(
    const bio_nav_interfaces::msg::PlanningPrior & prior);
  static RouteContext routeContextOf(
    const bio_nav_interfaces::msg::PlanningPrior & prior);
  static CognitiveObstacleLayer::Identity priorIdentityOf(
    const bio_nav_interfaces::msg::PlanningPrior & prior);
  bool obstacleOnlyScoring() const;

  std::mutex mutex_;
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr obstacles_;
  bio_nav_interfaces::msg::PlanningPrior::SharedPtr prior_;
  CognitiveObstacleLayer::Identity expected_;
  CognitiveObstacleLayer::AcceptanceCursor accepted_;
  bool identity_bound_{false};
  RouteContext route_context_;
  CognitiveObstacleLayer::Identity route_identity_;
  bool route_context_bound_{false};
  CognitiveObstacleLayer::Identity pending_rebind_identity_;
  bool pending_rebind_{false};
  RejectedOffer last_rejected_offer_;
  uint64_t last_status_sequence_{0};
  bool last_status_applied_{false};
  std::string last_status_reason_;
  bio_nav_interfaces::msg::RiskLayerStatus last_status_;
  std::string mode_{"off"};
  std::string obstacle_topic_{"/bio_nav/module2/cognitive_obstacles"};
  std::string prior_topic_{"/bio_nav/module2/planning_prior"};
  double maximum_age_s_{0.5};
  double maximum_ood_probability_{0.2};
  float obstacle_weight_{4.0F};
  float direction_weight_{1.0F};
  float novelty_weight_{0.5F};
  float uncertainty_weight_{0.5F};
  rclcpp::Subscription<
    bio_nav_interfaces::msg::CognitiveObstacleArray>::SharedPtr obstacle_subscription_;
  rclcpp::Subscription<
    bio_nav_interfaces::msg::PlanningPrior>::SharedPtr prior_subscription_;
  rclcpp::Publisher<bio_nav_interfaces::msg::RiskLayerStatus>::SharedPtr
    status_publisher_;
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__COGNITIVE_RISK_CRITIC_HPP_
