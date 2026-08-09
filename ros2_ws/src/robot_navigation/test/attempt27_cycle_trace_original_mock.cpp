#include "nav2_mppi_controller/controller.hpp"

namespace nav2_mppi_controller
{

geometry_msgs::msg::TwistStamped MPPIController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::Twist & robot_speed,
  nav2_core::GoalChecker *)
{
  geometry_msgs::msg::TwistStamped result;
  result.twist.linear.x = robot_pose.pose.position.x + robot_speed.linear.x;
  result.twist.angular.z = robot_pose.pose.position.y + robot_speed.angular.z;
  return result;
}

}  // namespace nav2_mppi_controller
