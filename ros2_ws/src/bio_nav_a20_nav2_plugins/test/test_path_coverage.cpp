#include <vector>

#include "bio_nav_a20_nav2_plugins/path_coverage.hpp"
#include "gtest/gtest.h"

namespace bio_nav_a20_nav2_plugins
{
namespace
{

std::vector<float> linspace(float start, float end, std::size_t count)
{
  std::vector<float> values(count);
  for (std::size_t i = 0; i < count; ++i) {
    values[i] = start + (end - start) * static_cast<float>(i) /
      static_cast<float>(count - 1);
  }
  return values;
}

TEST(PathCoverageTest, FullCoverageReturnsOne)
{
  const auto path_x = linspace(0.0f, 10.0f, 11);
  const std::vector<float> path_y(11, 0.0f);
  const auto traj_x = linspace(0.0f, 10.0f, 30);
  const std::vector<float> traj_y(30, 0.0f);

  const double coverage = compute_path_coverage(traj_x, traj_y, path_x, path_y);
  EXPECT_DOUBLE_EQ(coverage, 1.0);
}

TEST(PathCoverageTest, HalfwayTrajectoryCoversHalfThePath)
{
  const auto path_x = linspace(0.0f, 10.0f, 11);
  const std::vector<float> path_y(11, 0.0f);
  const auto traj_x = linspace(0.0f, 5.0f, 30);
  const std::vector<float> traj_y(30, 0.0f);

  const double coverage = compute_path_coverage(traj_x, traj_y, path_x, path_y);
  EXPECT_NEAR(coverage, 0.5, 1e-6);
}

TEST(PathCoverageTest, BackwardTrajectoryCoversNothing)
{
  const auto path_x = linspace(0.0f, 10.0f, 11);
  const std::vector<float> path_y(11, 0.0f);
  // Drives away from the path: every point projects to path point 0.
  const auto traj_x = linspace(0.0f, -2.0f, 30);
  const std::vector<float> traj_y(30, 0.0f);

  const double coverage = compute_path_coverage(traj_x, traj_y, path_x, path_y);
  EXPECT_DOUBLE_EQ(coverage, 0.0);
}

TEST(PathCoverageTest, CoverageIsDirectionAgnostic)
{
  // A reverse trajectory that still advances along the path arc length must
  // score exactly like a forward one: no forward-velocity bias exists here.
  const auto path_x = linspace(0.0f, 10.0f, 11);
  const std::vector<float> path_y(11, 0.0f);
  const auto traj_x = linspace(4.0f, 8.0f, 30);
  const std::vector<float> traj_y(30, 0.0f);

  const double coverage = compute_path_coverage(traj_x, traj_y, path_x, path_y);
  EXPECT_NEAR(coverage, 0.8, 1e-6);
}

TEST(PathCoverageTest, EmptyPathAndEmptyTrajectory)
{
  const std::vector<float> empty;
  const auto traj_x = linspace(0.0f, 1.0f, 5);
  const std::vector<float> traj_y(5, 0.0f);

  // No path: no progress demand, so no residual cost.
  EXPECT_DOUBLE_EQ(compute_path_coverage(traj_x, traj_y, empty, empty), 1.0);

  const auto path_x = linspace(0.0f, 10.0f, 11);
  const std::vector<float> path_y(11, 0.0f);
  // No trajectory points: nothing covered.
  EXPECT_DOUBLE_EQ(compute_path_coverage(empty, empty, path_x, path_y), 0.0);
}

TEST(PathCoverageTest, ZeroLengthPathIsFullyCovered)
{
  const std::vector<float> path_x(4, 3.0f);
  const std::vector<float> path_y(4, 3.0f);
  const auto traj_x = linspace(3.0f, 3.5f, 5);
  const std::vector<float> traj_y(5, 3.0f);

  EXPECT_DOUBLE_EQ(compute_path_coverage(traj_x, traj_y, path_x, path_y), 1.0);
}

TEST(PathCoverageTest, CoverageNeverExceedsOne)
{
  const auto path_x = linspace(0.0f, 10.0f, 11);
  const std::vector<float> path_y(11, 0.0f);
  // Overshoots the path end: still exactly full coverage.
  const auto traj_x = linspace(0.0f, 15.0f, 30);
  const std::vector<float> traj_y(30, 0.0f);

  EXPECT_DOUBLE_EQ(compute_path_coverage(traj_x, traj_y, path_x, path_y), 1.0);
}

}  // namespace
}  // namespace bio_nav_a20_nav2_plugins
