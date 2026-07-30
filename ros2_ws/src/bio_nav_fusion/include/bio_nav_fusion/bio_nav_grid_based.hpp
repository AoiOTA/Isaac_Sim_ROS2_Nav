#ifndef BIO_NAV_FUSION__BIO_NAV_GRID_BASED_HPP_
#define BIO_NAV_FUSION__BIO_NAV_GRID_BASED_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bio_nav_interfaces/msg/planner_decision.hpp"
#include "bio_nav_interfaces/msg/planning_prior.hpp"
#include "bio_nav_interfaces/srv/get_goal_planning_prior.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace bio_nav_fusion
{

struct GridSearchResult
{
  nav_msgs::msg::Path path;
  uint64_t expanded_nodes{0};
  double primary_cost{0.0};
  bool success{false};
  std::string error;
};

class BioNavGridBased : public nav2_core::GlobalPlanner
{
public:
  BioNavGridBased() = default;
  ~BioNavGridBased() override;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  static GridSearchResult equalCostSearch(
    nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::array<float, 256> & tie_break_score,
    bool allow_unknown,
    const std::string & global_frame,
    const rclcpp::Time & stamp,
    const std::function<bool()> & cancel_checker);
  static bool priorIdentityFresh(double age_s, double maximum_age_s);

private:
  void publishDecision(
    bool used, const std::string & fallback_reason, double primary_cost,
    uint64_t expanded_nodes, double latency_ms, uint32_t reset_epoch,
    const std::string & map_version, const std::string & goal_hash,
    const std::string & snapshot_sha256);
  nav_msgs::msg::Path stockPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::function<bool()> & cancel_checker,
    const std::string & reason, double begin_s);

  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("BioNavGridBased")};
  rclcpp::Clock::SharedPtr clock_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<nav2_smac_planner::SmacPlanner2D> stock_;
  rclcpp_lifecycle::LifecyclePublisher<
    bio_nav_interfaces::msg::PlannerDecision>::SharedPtr decision_publisher_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::thread client_thread_;
  rclcpp::Client<
    bio_nav_interfaces::srv::GetGoalPlanningPrior>::SharedPtr prior_client_;
  rclcpp::Subscription<
    bio_nav_interfaces::msg::PlanningPrior>::SharedPtr identity_subscription_;
  std::mutex identity_mutex_;
  bool identity_seen_{false};
  rclcpp::Time identity_stamp_{0, 0, RCL_ROS_TIME};
  uint32_t reset_epoch_{0};
  std::string map_version_;
  std::string service_name_{"/bio_nav/get_goal_planning_prior"};
  std::string prior_topic_{"/bio_nav/module2/planning_prior"};
  std::string planner_profile_{"bio_nav_tiebreak_risk"};
  std::string expected_module3_map_sha256_;
  std::string expected_qualification_sha256_;
  std::string qualification_sha256_;
  std::string motion_core_sha256_;
  bool allow_unknown_{true};
  int service_timeout_ms_{100};
  double maximum_prior_age_s_{0.5};
  std::atomic<uint64_t> sequence_{0};
};

}  // namespace bio_nav_fusion

#endif  // BIO_NAV_FUSION__BIO_NAV_GRID_BASED_HPP_
