#pragma once  // TODO: FIX

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#define DEBUG_ORM

namespace collision_avoidance
{
// Obstacle-Restriction Method
class ORM
{
private:
  double radius_;
  double security_distance_;
  double epsilon_;

public:
  ORM(double radius, double security_distance, double epsilon);

  Eigen::Vector2d avoidCollision(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

private:
  inline double radBounded(double value)
  {
    return std::remainder(value, 2.0 * M_PI);
  }

  Eigen::Vector2d subgoalSelector(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

  bool isSubgoal(const std::vector<Eigen::Vector2d>& obstacles, int index_1, int index_2, Eigen::Vector2d* subgoal);

  Eigen::Vector2d motionComputation(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

  std::pair<double, double> getBounds(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

  bool isPathClear(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

  void getPointsOfInterest(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles,
                           std::vector<Eigen::Vector2d>* left, std::vector<Eigen::Vector2d>* right);

  std::vector<Eigen::Vector2d> getRectangle(const Eigen::Vector2d& goal);

  std::vector<Eigen::Vector2d> getPointsInPolygon(const std::vector<Eigen::Vector2d>& polygon,
                                                  const std::vector<Eigen::Vector2d> points);

  bool isPointInPolygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point);
};
}  // namespace collision_avoidance
