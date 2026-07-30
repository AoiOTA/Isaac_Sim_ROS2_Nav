#ifndef BIO_NAV_FUSION__COGNITIVE_RISK_LAYER_HPP_
#define BIO_NAV_FUSION__COGNITIVE_RISK_LAYER_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "bio_nav_interfaces/msg/planning_prior.hpp"
#include "bio_nav_interfaces/msg/risk_layer_status.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/empty.hpp"

namespace bio_nav_fusion
{

class CognitiveRiskLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  CognitiveRiskLayer();

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  static uint8_t mapRiskCost(
    float probability, float threshold, double decay, int maximum_cost);
  static std::string validatePrior(
    const bio_nav_interfaces::msg::PlanningPrior * prior,
    double age_s, double maximum_age_s, double minimum_reliability,
    uint32_t reset_epoch, const std::string & expected_map_version,
    const std::string & expected_risk_model_sha256,
    const std::string & expected_qualification_sha256);

private:
  void priorCallback(const bio_nav_interfaces::msg::PlanningPrior::SharedPtr message);
  void resetCallback(const std_msgs::msg::Empty::SharedPtr message);
  bool validateLocked(
    const rclcpp::Time & now, std::string & reason, double & age_s) const;
  void publishStatus(
    bool applied, const std::string & reason, double age_s,
    uint32_t active_cells, uint8_t maximum_cost);

  std::mutex mutex_;
  bio_nav_interfaces::msg::PlanningPrior::SharedPtr latest_;
  uint32_t reset_epoch_{0};
  bool reset_epoch_initialized_{false};
  double max_message_age_s_{0.75};
  double minimum_reliability_{0.2};
  int maximum_cost_{80};
  std::string expected_map_version_;
  std::string expected_risk_model_sha256_;
  std::string expected_qualification_sha256_;
  std::string prior_topic_{"/bio_nav/module2/planning_prior"};
  std::string reset_topic_{"/simulation/reset_event"};
  rclcpp::Subscription<bio_nav_interfaces::msg::PlanningPrior>::SharedPtr prior_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<
    bio_nav_interfaces::msg::RiskLayerStatus>::SharedPtr status_publisher_;
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__COGNITIVE_RISK_LAYER_HPP_
