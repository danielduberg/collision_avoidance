#include <collision_avoidance/no_input.h>

namespace collision_avoidance
{
namespace no_input
{
Eigen::Vector2d avoidCollision(const std::vector<Eigen::Vector2d>& obstacles, double radius, double min_distance_hold)
{
  std::pair<double, double> x(-10000, 10000);
  std::pair<double, double> y(x);

  double closest_distance = radius + min_distance_hold;
  for (const Eigen::Vector2d& obstacle : obstacles)
  {
    if (!obstacle.allFinite())
    {
      continue;
    }

    double distance = obstacle.norm();
    if (distance < closest_distance)
    {
      double magnitude = closest_distance - distance;
      double direction = std::atan2(obstacle[1], obstacle[0]) + M_PI;  // We want to move in the opposite direction

      x = std::minmax({ x.first, x.second, magnitude * std::cos(direction) });
      y = std::minmax({ y.first, y.second, magnitude * std::sin(direction) });
    }
  }

  return Eigen::Vector2d(x.first + x.second, y.first + y.second) * 2.0;
}
}  // namespace no_input
}  // namespace collision_avoidance
