#ifndef COLLISION_AVOIDANCE_NO_INPUT_H
#define COLLISION_AVOIDANCE_NO_INPUT_H

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#include <Eigen/Dense>

namespace collision_avoidance
{
namespace no_input
{
Eigen::Vector2d avoidCollision(const std::vector<Eigen::Vector2d>& obstacles, double radius, double min_distance_hold);
}  // namespace no_input
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_NO_INPUT_H