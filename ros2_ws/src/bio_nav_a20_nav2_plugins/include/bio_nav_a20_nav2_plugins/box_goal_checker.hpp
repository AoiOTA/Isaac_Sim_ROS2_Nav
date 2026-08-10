#ifndef BIO_NAV_A20_NAV2_PLUGINS__BOX_GOAL_CHECKER_HPP_
#define BIO_NAV_A20_NAV2_PLUGINS__BOX_GOAL_CHECKER_HPP_

#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bio_nav_a20_nav2_plugins
{

/**
 * @brief Pure box-tolerance test in the goal frame.
 *
 * Transforms the query pose into the goal (reference) frame and checks
 * |dx| <= longitudinal_tol along the goal x axis, |dy| <= lateral_tol along
 * the goal y axis and the absolute wrapped yaw difference against yaw_tol.
 * A yaw_tol of pi accepts every orientation (yaw-free goal).
 */
inline bool is_within_goal_box(
  double query_x, double query_y, double query_yaw,
  double goal_x, double goal_y, double goal_yaw,
  double longitudinal_tol, double lateral_tol, double yaw_tol)
{
  const double dx = query_x - goal_x;
  const double dy = query_y - goal_y;
  const double cos_goal = std::cos(goal_yaw);
  const double sin_goal = std::sin(goal_yaw);
  const double longitudinal = cos_goal * dx + sin_goal * dy;
  const double lateral = -sin_goal * dx + cos_goal * dy;
  double dyaw = std::remainder(query_yaw - goal_yaw, 2.0 * M_PI);
  return std::fabs(longitudinal) <= longitudinal_tol &&
         std::fabs(lateral) <= lateral_tol &&
         std::fabs(dyaw) <= yaw_tol;
}

/**
 * @class bio_nav_a20_nav2_plugins::BoxGoalChecker
 * @brief Goal checker with separate longitudinal/lateral tolerances in the
 * goal frame and an independently set yaw tolerance.
 *
 * Matches the point-goal tolerance mismatch seen in A19: a portal crossing
 * only needs tight tolerance along the crossing direction, while the lateral
 * band and the heading stay free. stateful semantics follow
 * nav2_controller::SimpleGoalChecker: with stateful=true the xy box is not
 * re-checked once satisfied; with stateful=false every call re-evaluates the
 * full box. The default is stateful=false.
 */
class BoxGoalChecker : public nav2_core::GoalChecker
{
public:
  BoxGoalChecker();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

protected:
  double xy_goal_tolerance_longitudinal_{0.05};
  double xy_goal_tolerance_lateral_{0.15};
  double yaw_goal_tolerance_{M_PI};
  bool stateful_{false};
  bool check_xy_{true};
  std::string plugin_name_;
};

}  // namespace bio_nav_a20_nav2_plugins

#endif  // BIO_NAV_A20_NAV2_PLUGINS__BOX_GOAL_CHECKER_HPP_
