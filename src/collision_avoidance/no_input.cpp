#include <collision_avoidance/no_input.h>

namespace collision_avoidance
{
NoInput::NoInput() {}

void NoInput::avoidCollision(geometry_msgs::TwistStamped* control,
                             const std::vector<Point>& obstacles, double radius,
                             double min_distance_hold)
{
  double x_min = 10000;
  double y_min = 10000;
  double x_max = -10000;
  double y_max = -10000;

  for (size_t i = 0; i < obstacles.size(); ++i)
  {
    if (!Point::isfinite(obstacles[i]))
    {
      // No reading here
      continue;
    }

    double distance = Point::getDistance(obstacles[i]);

    if (distance <= radius + min_distance_hold)
    {
      double direction = Point::getDirectionDegrees(obstacles[i]) + 180.0;
      if (direction >= 360)
      {
        direction -= 360;
      }

      double magnitude = (radius + min_distance_hold) - distance;

      Point p = Point::getPointFromVectorDegrees(direction, magnitude);

      x_min = std::min(x_min, p.x_);
      x_max = std::max(x_max, p.x_);
      y_min = std::min(y_min, p.y_);
      y_max = std::max(y_max, p.y_);
    }
  }

  control->twist.linear.x = (x_min + x_max) / 2.0;
  control->twist.linear.y = (y_min + y_max) / 2.0;
}
}
