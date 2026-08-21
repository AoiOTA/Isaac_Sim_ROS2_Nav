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
  static std::string validateDirectionPrior(
    const bio_nav_interfaces::msg::PlanningPrior & prior);

private:
  friend class CognitiveRiskCriticTestPeer;

  void obstacleCallback(
    const bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr message);
  void priorCallback(
    const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message);
  void publishStatus(uint64_t sequence, bool applied, const std::string & reason);

  std::mutex mutex_;
  bio_nav_interfaces::msg::CognitiveObstacleArray::SharedPtr obstacles_;
  bio_nav_interfaces::msg::PlanningPrior::SharedPtr prior_;
  bio_nav_interfaces::msg::PlanningPrior::SharedPtr accepted_prior_;
  CognitiveObstacleLayer::Identity expected_;
  CognitiveObstacleLayer::AcceptanceCursor accepted_;
  bool identity_bound_{false};
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
