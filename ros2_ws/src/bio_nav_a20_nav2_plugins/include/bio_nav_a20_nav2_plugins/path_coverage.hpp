#ifndef BIO_NAV_A20_NAV2_PLUGINS__PATH_COVERAGE_HPP_
#define BIO_NAV_A20_NAV2_PLUGINS__PATH_COVERAGE_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace bio_nav_a20_nav2_plugins
{

// Pure path-coverage math shared by TubeProgressCritic and its gtest.
// All containers only need size() and operator[] (xtensor views qualify), so
// the functions impose no copies and no ROS dependency.

// Cumulative arc length of a polyline: s[0] = 0, s[i] = s[i-1] + |p[i]-p[i-1]|.
template<typename XContainer, typename YContainer>
inline std::vector<float> compute_cumulative_arclength(
  const XContainer & path_x, const YContainer & path_y)
{
  const std::size_t n = path_x.size();
  std::vector<float> arclength(n, 0.0f);
  for (std::size_t i = 1; i < n; ++i) {
    const float dx = static_cast<float>(path_x[i]) - static_cast<float>(path_x[i - 1]);
    const float dy = static_cast<float>(path_y[i]) - static_cast<float>(path_y[i - 1]);
    arclength[i] = arclength[i - 1] + std::hypot(dx, dy);
  }
  return arclength;
}

// Maximum arc-length coordinate reached by a trajectory: every trajectory
// point is projected onto its nearest path point and the largest projected
// arc length wins. Projecting every point (not only the endpoint) prevents a
// corner-cutting trajectory from claiming coverage it never drove along the
// path. The result is clamped to [0, path_arclength.back()].
template<typename TrajX, typename TrajY, typename PathX, typename PathY>
inline double compute_covered_arclength(
  const TrajX & traj_x, const TrajY & traj_y,
  const PathX & path_x, const PathY & path_y,
  const std::vector<float> & path_arclength)
{
  const std::size_t n_path = path_x.size();
  const std::size_t n_traj = traj_x.size();
  if (n_path == 0 || n_traj == 0 || path_arclength.empty()) {
    return 0.0;
  }

  double covered = 0.0;
  for (std::size_t t = 0; t < n_traj; ++t) {
    const float tx = static_cast<float>(traj_x[t]);
    const float ty = static_cast<float>(traj_y[t]);
    std::size_t nearest = 0;
    float best = std::numeric_limits<float>::infinity();
    for (std::size_t p = 0; p < n_path; ++p) {
      const float dx = tx - static_cast<float>(path_x[p]);
      const float dy = ty - static_cast<float>(path_y[p]);
      const float dist_sq = dx * dx + dy * dy;
      if (dist_sq < best) {
        best = dist_sq;
        nearest = p;
      }
    }
    covered = std::max(covered, static_cast<double>(path_arclength[nearest]));
  }
  return std::clamp(covered, 0.0, static_cast<double>(path_arclength.back()));
}

// Fraction of the path arc length covered by the trajectory, in [0, 1].
// A trajectory covering the full path returns 1.0. An empty path carries no
// progress demand and therefore returns 1.0; an empty trajectory covers
// nothing and returns 0.0.
template<typename TrajX, typename TrajY, typename PathX, typename PathY>
inline double compute_path_coverage(
  const TrajX & traj_x, const TrajY & traj_y,
  const PathX & path_x, const PathY & path_y)
{
  if (path_x.size() == 0) {
    return 1.0;
  }
  if (traj_x.size() == 0) {
    return 0.0;
  }
  const std::vector<float> arclength = compute_cumulative_arclength(path_x, path_y);
  if (arclength.back() <= 0.0f) {
    return 1.0;
  }
  return compute_covered_arclength(traj_x, traj_y, path_x, path_y, arclength) /
         static_cast<double>(arclength.back());
}

}  // namespace bio_nav_a20_nav2_plugins

#endif  // BIO_NAV_A20_NAV2_PLUGINS__PATH_COVERAGE_HPP_
