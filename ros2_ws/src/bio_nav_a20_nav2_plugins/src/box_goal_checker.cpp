#include "bio_nav_a20_nav2_plugins/box_goal_checker.hpp"

#include <limits>
#include <string>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bio_nav_a20_nav2_plugins
{

BoxGoalChecker::BoxGoalChecker() = default;

void BoxGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance_longitudinal",
    rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".xy_goal_tolerance_lateral",
    rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".yaw_goal_tolerance",
    rclcpp::ParameterValue(M_PI));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".stateful", rclcpp::ParameterValue(false));

  node->get_parameter(
    plugin_name + ".xy_goal_tolerance_longitudinal",
    xy_goal_tolerance_longitudinal_);
  node->get_parameter(
    plugin_name + ".xy_goal_tolerance_lateral", xy_goal_tolerance_lateral_);
  node->get_parameter(plugin_name + ".yaw_goal_tolerance", yaw_goal_tolerance_);
  node->get_parameter(plugin_name + ".stateful", stateful_);
}

void BoxGoalChecker::reset()
{
  check_xy_ = true;
}

bool BoxGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  const double query_yaw = tf2::getYaw(query_pose.orientation);
  const double goal_yaw = tf2::getYaw(goal_pose.orientation);

  if (stateful_ && !check_xy_) {
    // The xy box was satisfied earlier; only the yaw band remains.
    return is_within_goal_box(
      goal_pose.position.x, goal_pose.position.y, query_yaw,
      goal_pose.position.x, goal_pose.position.y, goal_yaw,
      xy_goal_tolerance_longitudinal_, xy_goal_tolerance_lateral_,
      yaw_goal_tolerance_);
  }

  const bool in_box = is_within_goal_box(
    query_pose.position.x, query_pose.position.y, query_yaw,
    goal_pose.position.x, goal_pose.position.y, goal_yaw,
    xy_goal_tolerance_longitudinal_, xy_goal_tolerance_lateral_,
    yaw_goal_tolerance_);
  if (stateful_ && in_box) {
    check_xy_ = false;
  }
  return in_box;
}

bool BoxGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  const double invalid_field = std::numeric_limits<double>::lowest();
  pose_tolerance.position.x = xy_goal_tolerance_longitudinal_;
  pose_tolerance.position.y = xy_goal_tolerance_lateral_;
  pose_tolerance.position.z = invalid_field;
  tf2::Quaternion yaw_quat;
  yaw_quat.setRPY(0.0, 0.0, yaw_goal_tolerance_);
  pose_tolerance.orientation = tf2::toMsg(yaw_quat);
  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;
  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;
  return true;
}

}  // namespace bio_nav_a20_nav2_plugins

PLUGINLIB_EXPORT_CLASS(
  bio_nav_a20_nav2_plugins::BoxGoalChecker, nav2_core::GoalChecker)
