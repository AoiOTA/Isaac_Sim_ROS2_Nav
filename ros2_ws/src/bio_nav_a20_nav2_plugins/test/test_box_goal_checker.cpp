#include <cmath>

#include "bio_nav_a20_nav2_plugins/box_goal_checker.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gtest/gtest.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bio_nav_a20_nav2_plugins
{
namespace
{

constexpr double kLon = 0.05;
constexpr double kLat = 0.15;
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Pose make_pose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  pose.orientation = tf2::toMsg(quat);
  return pose;
}

TEST(GoalBoxTest, InsideBandIsAccepted)
{
  EXPECT_TRUE(is_within_goal_box(
      0.04, -0.10, 2.9, 0.0, 0.0, 0.0, kLon, kLat, kPi));
}

TEST(GoalBoxTest, OutsideLongitudinalIsRejected)
{
  EXPECT_FALSE(is_within_goal_box(
      0.051, 0.0, 0.0, 0.0, 0.0, 0.0, kLon, kLat, kPi));
  EXPECT_FALSE(is_within_goal_box(
      -0.051, 0.0, 0.0, 0.0, 0.0, 0.0, kLon, kLat, kPi));
}

TEST(GoalBoxTest, OutsideLateralIsRejected)
{
  EXPECT_FALSE(is_within_goal_box(
      0.0, 0.151, 0.0, 0.0, 0.0, 0.0, kLon, kLat, kPi));
  EXPECT_FALSE(is_within_goal_box(
      0.0, -0.151, 0.0, 0.0, 0.0, 0.0, kLon, kLat, kPi));
}

TEST(GoalBoxTest, BoundaryIsInclusive)
{
  EXPECT_TRUE(is_within_goal_box(
      kLon, kLat, 0.0, 0.0, 0.0, 0.0, kLon, kLat, kPi));
  EXPECT_TRUE(is_within_goal_box(
      -kLon, -kLat, 0.0, 0.0, 0.0, 0.0, kLon, kLat, kPi));
}

TEST(GoalBoxTest, BoxRotatesWithGoalFrame)
{
  const double goal_yaw = kPi / 2.0;
  // +0.04 m along world y is +0.04 m along the goal's longitudinal axis.
  EXPECT_TRUE(is_within_goal_box(
      1.0, 1.04, 0.0, 1.0, 1.0, goal_yaw, kLon, kLat, kPi));
  // The same world-frame displacement along x is lateral in the goal frame
  // and exceeds the longitudinal band when applied longitudinally there.
  EXPECT_FALSE(is_within_goal_box(
      1.10, 1.0, 0.0, 1.0, 1.0, goal_yaw, 0.05, 0.05, kPi));
  EXPECT_TRUE(is_within_goal_box(
      1.10, 1.0, 0.0, 1.0, 1.0, goal_yaw, kLon, kLat, kPi));
}

TEST(GoalBoxTest, YawToleranceIsIndependentAndWraps)
{
  // Tight yaw band rejects a 0.2 rad heading error.
  EXPECT_FALSE(is_within_goal_box(
      0.0, 0.0, 0.2, 0.0, 0.0, 0.0, kLon, kLat, 0.1));
  // Wrapped difference across +/-pi is 0.1 rad and passes a 0.15 band.
  EXPECT_TRUE(is_within_goal_box(
      0.0, 0.0, -kPi + 0.05, 0.0, 0.0, kPi - 0.05, kLon, kLat, 0.15));
  // A yaw tolerance of pi accepts any heading.
  EXPECT_TRUE(is_within_goal_box(
      0.0, 0.0, kPi, 0.0, 0.0, 0.0, kLon, kLat, kPi));
}

TEST(BoxGoalCheckerTest, DefaultCheckerAppliesBoxInGoalFrame)
{
  // The default-constructed plugin already carries the documented defaults
  // (0.05 longitudinal / 0.15 lateral / pi yaw, stateful=false), so the
  // reached check can run without a node.
  BoxGoalChecker checker;
  const geometry_msgs::msg::Twist velocity;

  EXPECT_TRUE(checker.isGoalReached(
      make_pose(0.04, 0.14, 2.0), make_pose(0.0, 0.0, 0.0), velocity));
  EXPECT_FALSE(checker.isGoalReached(
      make_pose(0.06, 0.0, 0.0), make_pose(0.0, 0.0, 0.0), velocity));
  EXPECT_FALSE(checker.isGoalReached(
      make_pose(0.0, 0.16, 0.0), make_pose(0.0, 0.0, 0.0), velocity));
  // Yaw is free at the default pi tolerance.
  EXPECT_TRUE(checker.isGoalReached(
      make_pose(0.0, 0.0, -3.0), make_pose(0.0, 0.0, 1.5), velocity));
}

}  // namespace
}  // namespace bio_nav_a20_nav2_plugins
