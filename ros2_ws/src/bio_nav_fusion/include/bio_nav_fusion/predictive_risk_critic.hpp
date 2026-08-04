#ifndef BIO_NAV_FUSION__PREDICTIVE_RISK_CRITIC_HPP_
#define BIO_NAV_FUSION__PREDICTIVE_RISK_CRITIC_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "bio_nav_interfaces/msg/local_critic_audit.hpp"
#include "bio_nav_interfaces/msg/predictive_risk_grid.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace mppi::critics
{

/// Attempt-22 predictive-risk scorer. It is non-mutating in Shadow mode.
class PredictiveRiskCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

  static std::string validateRisk(
    const bio_nav_interfaces::msg::PredictiveRiskGrid * grid,
    double age_s, double maximum_age_s, double minimum_reliability,
    double maximum_ood_probability, uint32_t reset_epoch,
    const std::string & expected_frame,
    const std::string & expected_map_version,
    const std::string & expected_model_sha256,
    const std::string & expected_calibration_sha256,
    const std::string & expected_qualification_sha256);

  static float sampleRisk(
    const bio_nav_interfaces::msg::PredictiveRiskGrid & grid,
    float local_x, float local_y, float time_s);

private:
  void riskCallback(
    const bio_nav_interfaces::msg::PredictiveRiskGrid::SharedPtr message);

  std::mutex mutex_;
  bio_nav_interfaces::msg::PredictiveRiskGrid::SharedPtr latest_;
  rclcpp::Subscription<bio_nav_interfaces::msg::PredictiveRiskGrid>::SharedPtr
    risk_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<
    bio_nav_interfaces::msg::LocalCriticAudit>::SharedPtr audit_publisher_;
  uint64_t sequence_{0};
  uint32_t reset_epoch_{0};
  bool shadow_only_{true};
  bool active_authorized_{false};
  double maximum_age_s_{0.3};
  double minimum_reliability_{0.6};
  double maximum_ood_probability_{0.4};
  float risk_weight_{0.5F};
  float maximum_standard_cost_{100000.0F};
  std::string risk_topic_{"/bio_nav/module2/predictive_risk_grid"};
  std::string audit_topic_{"/bio_nav/module3/predictive_risk_critic_audit"};
  std::string expected_frame_{"base_link"};
  std::string expected_map_version_;
  std::string expected_model_sha256_;
  std::string expected_calibration_sha256_;
  std::string expected_qualification_sha256_;
};

}  // namespace mppi::critics

#endif  // BIO_NAV_FUSION__PREDICTIVE_RISK_CRITIC_HPP_
