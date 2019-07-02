#pragma once

#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>

#include <collision_avoidance/point.h>

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
  ORM(double radius, double security_distance, double epsilon, double max_change_in_direction);

  Eigen::Vector2d avoidCollision(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

private:
  Eigen::Vector2d subgoalSelector(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles);

  Eigen::Vector2d motionComputation(const Eigen::Vector2d goal, const std::vector<Eigen::Vector2d>& obstacles);

public:
  static bool avoidCollision(ros::Publisher& pub_, geometry_msgs::TwistStamped* control, const double magnitude,
                             const std::vector<Point>& obstacles, double radius, double security_distance,
                             double epsilon, double min_distance_hold, double min_change_in_direction,
                             double max_change_in_direction, double min_opposite_direction,
                             double max_opposite_direction);

private:
  static Point initGoal(double x, double y);

  static bool isSubgoal(ros::Publisher& pub_, int index, int previous_index, Point* subgoal,
                        const std::vector<Point>& L, double radius, double security_distance, double epsilon);

  static Point subgoalSelector(ros::Publisher& pub_, const Point& goal, double magnitude, const std::vector<Point>& L,
                               double radius, double security_distance, double epsilon, double min_change_in_direction,
                               double max_change_in_direction, double min_opposite_direction,
                               double max_opposite_direction);

  // TODO: Take an extra look at this one
  static void getPointsOfInterest(const Point& goal, const std::vector<Point>& L, std::vector<Point>* left,
                                  std::vector<Point>* right, double radius, double security_distance,
                                  double max_angle = 180,
                                  double max_distance = std::numeric_limits<double>::infinity());

  static double getLeftBound(const std::vector<Point>& L, double goal_direction, double radius,
                             double security_distance);

  static double getRightBound(const std::vector<Point>& L, double goal_direction, double radius,
                              double security_distance);

  static Point motionComputation(const Point& goal, const std::vector<Point>& L, double radius,
                                 double security_distance);

  static bool isPointInPolygon(const Point& point, const std::vector<Point>& verticies);

  static std::vector<Point> getPointsInPolygon(const std::vector<Point>& L, const std::vector<Point>& verticies);

  static void getRectangle(const Point& goal, double radius, std::vector<Point>* verticies);

  static bool isClearPath(ros::Publisher& pub_, const Point& goal, const std::vector<Point>& L, double radius,
                          double security_distance);

  static double getMidDirection(double d1, double d2);
};
}  // namespace collision_avoidance
