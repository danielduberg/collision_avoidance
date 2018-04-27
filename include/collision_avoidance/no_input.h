#pragma once

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#include <collision_avoidance/point.h>

namespace collision_avoidance
{

class NoInput
{
public:
  static void avoidCollision(geometry_msgs::TwistStamped* control,
                             const std::vector<Point>& obstacles, double radius,
                             double min_distance_hold);

private:
  // Disallow creating an instance of this object
  NoInput();
};
}
