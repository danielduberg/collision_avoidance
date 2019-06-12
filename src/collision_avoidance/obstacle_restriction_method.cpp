#include <collision_avoidance/obstacle_restriction_method.h>
#include <collision_avoidance/no_input.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace collision_avoidance
{
int mod(int a, int b) { return ((a %= b) < 0) ? a + b : a; }

double radiansToDegrees(double r)
{
  r *= 180.0 / M_PI;

  if (r < 0)
  {
    return (r + 360);
  }

  return r;
}

ORM::ORM() {}

bool ORM::avoidCollision(ros::Publisher& pub_,
    geometry_msgs::TwistStamped* controller, const double magnitude,
    const std::vector<Point>& obstacles, double radius,
    double security_distance, double epsilon, double min_distance_hold,
    double min_change_in_direction, double max_change_in_direction,
    double min_opposite_direction, double max_opposite_direction)
{
  geometry_msgs::TwistStamped controller_original = *controller;

  if (controller->twist.linear.x == 0 && controller->twist.linear.y == 0)
  {
    // Nothing to do!
    // TODO: No input?
    NoInput::avoidCollision(controller, obstacles, radius, min_distance_hold);
    return true;
  }

  Point goal(controller->twist.linear.x, controller->twist.linear.y);

  // A. The Subgoal Selector
  goal =
      subgoalSelector(pub_, goal, magnitude, obstacles, radius, security_distance,
                      epsilon, min_change_in_direction, max_change_in_direction,
                      min_opposite_direction, max_opposite_direction);

  controller->twist.linear.x = goal.x_;
  controller->twist.linear.y = goal.y_;

  if (goal.x_ == 0 && goal.y_ == 0)
  {
    // Could not find any good command :(
    // TODO: No input?
    NoInput::avoidCollision(controller, obstacles, radius, min_distance_hold);
    return false;
  }

  // B. Motion Computation
  goal = motionComputation(goal, obstacles, radius, security_distance);

  controller->twist.linear.x = goal.x_;
  controller->twist.linear.y = goal.y_;

  Point goal_original(controller_original.twist.linear.x,
                      controller_original.twist.linear.y);

  double angular_diff = std::fabs(Point::getDirectionDegrees(goal_original) -
                                  Point::getDirectionDegrees(goal));

  if (angular_diff > 180)
  {
    angular_diff = std::fabs(angular_diff - 360);
  }

  ////ROS_FATAL("%f", angular_diff);

  if (angular_diff > max_change_in_direction)
  {
    controller->twist.linear.x = 0;
    controller->twist.linear.y = 0;

    std::cout << "Angle diff too large! " << ros::Time::now() << std::endl;

    NoInput::avoidCollision(controller, obstacles, radius, min_distance_hold);
    return false;
  }

  return true;
}

Point ORM::initGoal(double x, double y) { return Point(x, y); }

bool ORM::isSubgoal(ros::Publisher& pub_, int index, int previous_index, Point* subgoal,
                    const std::vector<Point>& L, double radius,
                    double security_distance, double epsilon)
{
  if (Point::isnan(L[index]) || Point::isnan(L[previous_index]))
  {
    // Not a subgoal because we do not know what is there
    return false;
  }

  if (Point::isinf(L[index]) && Point::isinf(L[previous_index]))
  {
    // Not a subgoal because it is only open space
    return false;
  }

  double degrees_per_index = 360.0 / L.size();

  if (Point::isinf(L[index]) && Point::isfinite(L[previous_index]))
  {
    // Found subgoal at the edge of an obstacle
    double direction = getMidDirection(index * degrees_per_index,
                                       previous_index * degrees_per_index);
    double distance = Point::getDistance(L[index]) + (radius * 2.0) + epsilon;

    Point temp_subgoal = Point::getPointFromVectorDegrees(direction, distance);

    if (isClearPath(pub_, temp_subgoal, L, radius, security_distance))
    {
      *subgoal = temp_subgoal;
      return true;
    }
  }

  if (Point::isfinite(L[index]) && Point::isinf(L[previous_index]))
  {
    // Found subgoal at the edge of an obstacle
    double direction = getMidDirection(index * degrees_per_index,
                                       previous_index * degrees_per_index);
    double distance =
        Point::getDistance(L[previous_index]) + (radius * 2.0) + epsilon;

    Point temp_subgoal = Point::getPointFromVectorDegrees(direction, distance);

    if (isClearPath(pub_, temp_subgoal, L, radius, security_distance))
    {
      *subgoal = temp_subgoal;
      return true;
    }
  }

  // We know that both are finite now

  if (Point::getDistance(L[index], L[previous_index]) > 2.0 * radius)
  {
    // Found subgoal between two obstacles
    Point temp_subgoal = Point::getMidpoint(L[index], L[previous_index]);

    if (isClearPath(pub_, temp_subgoal, L, radius, security_distance))
    {
      *subgoal = temp_subgoal;
      return true;
    }
  }

  // Did not find a subgoal
  return false;
}

Point ORM::subgoalSelector(ros::Publisher& pub_, const Point& goal, double magnitude,
                           const std::vector<Point>& L, double radius,
                           double security_distance, double epsilon,
                           double min_change_in_direction,
                           double max_change_in_direction,
                           double min_opposite_direction,
                           double max_opposite_direction)
{
  if (isClearPath(pub_, goal, L, radius, security_distance))
  {
    // We can go where we want
    return goal;
  }

  double degrees_per_index = 360.0 / L.size();

  int wanted_index = Point::getDirectionDegrees(goal) / degrees_per_index;

  double change_in_direction =
      ((max_change_in_direction - min_change_in_direction) * magnitude) +
      min_change_in_direction; // 180;
  double opposite_direction =
      ((max_opposite_direction - min_opposite_direction) * (1.0 - magnitude)) +
      min_opposite_direction; // 0;

  //ROS_FATAL("Change: %f, Opposite: %f", change_in_direction,
  //          opposite_direction);

  Point subgoal;
  int subgoal_i = -1;

  bool found_right_subgoal = false;
  bool found_left_subgoal = false;

  // We cannot go directly towards where we "want", so find a subgoal
  // Look for subgoals close to goal
  for (size_t i = 1; i * degrees_per_index < change_in_direction; ++i)
  {
    if (found_right_subgoal && found_left_subgoal)
    {
      // We have sub-goals on both sides so we do not know which one to pick
      break;
    }

    if (subgoal_i != -1 &&
        (i - subgoal_i) * degrees_per_index > opposite_direction)
    {
      // Found sub_goal and it is allowed
      return subgoal;
    }

    if (!found_right_subgoal)
    {
      int right_index = mod(wanted_index + i, L.size());
      int right_previous_index = mod(wanted_index + (i - 1), L.size());

      if (isSubgoal(pub_, right_index, right_previous_index, &subgoal, L, radius,
                    security_distance, epsilon))
      {
        subgoal_i = i;
        found_right_subgoal = true;
      }
    }

    if (!found_left_subgoal)
    {
      int left_index = mod(wanted_index - i, L.size());
      int left_previous_index = mod(wanted_index - (i - 1), L.size());

      if (isSubgoal(pub_, left_index, left_previous_index, &subgoal, L, radius,
                    security_distance, epsilon))
      {
        subgoal_i = i;
        found_left_subgoal = true;
      }
    }
  }

  if (found_right_subgoal != found_left_subgoal)
  {
    // Found a good subgoal
    return subgoal;
  }

  // No subgoal found
  return goal;
}

bool ORM::isClearPath(ros::Publisher& pub_, const Point& goal, const std::vector<Point>& L,
                      double radius, double security_distance)
{
  // A contains points on the left side of goal
  // B contains points on the right side of goal
  std::vector<Point> A, B;

  getPointsOfInterest(goal, L, &A, &B, radius, security_distance, 30);

  // Corners of the rectangle (tunnel)
  std::vector<Point> rectangle;
  // Point a, b, c, d;

  // 2 * radius because then all obstacles that matters will be inside the
  // rectangle
  // getRectangle(goal, 2.0d * radius_, &a, &b, &c, &d);
  getRectangle(goal, 2.0 * radius, &rectangle);

  A = getPointsInPolygon(A, rectangle); // getPointsInRectangle(A, a, b, c, d);
  B = getPointsInPolygon(B, rectangle); // getPointsInRectangle(B, a, b, c, d);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.header.frame_id = "base_link";
  pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
  {
      pcl::PointXYZI point;
      point.x = goal.x_;
      point.y = goal.y_;
      point.z = 0.0;
      point.intensity = 0.5;
      cloud.points.push_back(point);
  }
  for (size_t i = 0; i < A.size(); ++i)
  {
      pcl::PointXYZI point;
      point.x = A[i].x_;
      point.y = A[i].y_;
      point.z = 0.0;
      point.intensity = 0.2;
      cloud.points.push_back(point);
  }
  for (size_t i = 0; i < B.size(); ++i)
  {
      pcl::PointXYZI point;
      point.x = B[i].x_;
      point.y = B[i].y_;
      point.z = 0.0;
      point.intensity = 0.65;
      cloud.points.push_back(point);
  }
  pub_.publish(cloud);

  // Check if path is clear
  for (size_t i = 0; i < A.size(); ++i)
  {
    for (size_t j = 0; j < B.size(); ++j)
    {
      if (Point::getDistance(A[i], B[j]) < 2.0 * radius)
      {
        return false;
      }
    }
  }

  return true;
}

// Source: http://stackoverflow.com/questions/2825412/draw-a-parallel-line
void ORM::getRectangle(const Point& goal, double radius,
                       std::vector<Point>* verticies)
{
  /*
  double x1 = 0.0d;
  double x2 = goal.x_;
  double y1 = 0.0d;
  double y2 = goal.y_;

  double L = distance...;

  */

  double distance = Point::getDistance(goal);

  double dx = goal.y_ / distance;
  double dy = -goal.x_ / distance;

  Point a(radius * dx, radius * dy);
  Point b(goal.x_ + a.x_, goal.y_ + a.y_);
  Point c(goal.x_ - a.x_, goal.y_ - a.y_);
  Point d(-a.x_, -a.y_);

  verticies->push_back(a);
  verticies->push_back(b);
  verticies->push_back(c);
  verticies->push_back(d);
}

std::vector<Point> ORM::getPointsInPolygon(const std::vector<Point>& L,
                                           const std::vector<Point>& verticies)
{
  std::vector<Point> Lp;

  for (size_t i = 0; i < L.size(); ++i)
  {
    if (!Point::isfinite(L[i]))
    {
      // No possible to be inside polygon
      continue;
    }

    // Is point inside polygon?
    if (isPointInPolygon(L[i], verticies))
    {
      // Inside polygon
      Lp.push_back(L[i]);
    }
  }

  return Lp;
}

// Source:
// https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
bool ORM::isPointInPolygon(const Point& point,
                           const std::vector<Point>& verticies)
{
  int i;
  int j;
  bool c = false;
  for (i = 0, j = verticies.size() - 1; i < verticies.size(); j = i++)
  {
    if (((verticies[i].y_ > point.y_) != (verticies[j].y_ > point.y_)) &&
        (point.x_ < (verticies[j].x_ - verticies[i].x_) *
                            (point.y_ - verticies[i].y_) /
                            (verticies[j].y_ - verticies[i].y_) +
                        verticies[i].x_))
    {
      c = !c;
    }
  }

  return c;
}

double ORM::getMidDirection(double d1, double d2)
{
  double mid_direction = (d1 + d2) / 2.0;

  if (std::fabs(d1 - d2) <= 180)
  {
    return mid_direction;
  }

  if (mid_direction >= 180)
  {
    return (mid_direction - 180);
  }

  return (mid_direction + 180);
}

Point ORM::motionComputation(const Point& goal, const std::vector<Point>& L,
                             double radius, double security_distance)
{
  std::vector<Point> left;
  std::vector<Point> right;

  getPointsOfInterest(goal, L, &left, &right, 180,
                      Point::getDistance(goal) + radius);

  double goal_direction = Point::getDirectionDegrees(goal);

  if (left.size() != 0 && right.size() != 0)
  {
    // ROS_FATAL("Hello");
    // Get left and right bounds
    double left_bound =
        getLeftBound(right, goal_direction, radius, security_distance);
    double right_bound =
        getRightBound(left, goal_direction, radius, security_distance);

    double diff_left = left_bound - goal_direction;
    double diff_right = right_bound - goal_direction;

    if (diff_left < 0)
    {
      diff_left += 360;
    }
    if (diff_right < 0)
    {
      diff_right += 360;
    }

    // TODO: Have to check under 360?!
    if (diff_left > 180 && diff_left < 360 && diff_right > 0 &&
        diff_right < 180)
    {
      return Point::getPointFromVectorDegrees(goal_direction,
                                              Point::getDistance(goal));
    }
    else if (diff_right > 0 && diff_right < 180 && diff_right > diff_left)
    {
      return Point::getPointFromVectorDegrees(left_bound,
                                              Point::getDistance(goal));
    }
    else if (diff_left > 180 && diff_left < 360 && diff_right > diff_left)
    {
      return Point::getPointFromVectorDegrees(right_bound,
                                              Point::getDistance(goal));
    }
    else
    {
      // ROS_FATAL("Middle");
      return Point::getPointFromVectorDegrees(
          getMidDirection(left_bound, right_bound), Point::getDistance(goal));
    }
  }
  else if (right.size() != 0)
  {
    double left_bound =
        getLeftBound(right, goal_direction, radius, security_distance);

    double diff_left = left_bound - goal_direction;

    if (diff_left < 0)
    {
      diff_left += 360;
    }

    if (diff_left > 180 && diff_left < 360)
    {
      return Point::getPointFromVectorDegrees(goal_direction,
                                              Point::getDistance(goal));
    }
    else
    {
      return Point::getPointFromVectorDegrees(left_bound,
                                              Point::getDistance(goal));
    }
  }
  else if (left.size() != 0)
  {
    double right_bound =
        getRightBound(left, goal_direction, radius, security_distance);

    double diff_right = right_bound - goal_direction;

    if (diff_right < 0)
    {
      diff_right += 360;
    }

    if (diff_right > 0 && diff_right < 180)
    {
      return Point::getPointFromVectorDegrees(goal_direction,
                                              Point::getDistance(goal));
    }
    else
    {
      return Point::getPointFromVectorDegrees(right_bound,
                                              Point::getDistance(goal));
    }
  }
  else
  {
    return Point::getPointFromVectorDegrees(goal_direction,
                                            Point::getDistance(goal));
  }
}

// Merge with findPotentialAB
void ORM::getPointsOfInterest(const Point& goal, const std::vector<Point>& L,
                              std::vector<Point>* left,
                              std::vector<Point>* right, double radius,
                              double security_distance, double max_angle,
                              double max_distance)
{
  double degrees_per_index = 360.0 / L.size();

  int wanted_index = Point::getDirectionDegrees(goal) / degrees_per_index;

  double magnitude_nan = radius + (security_distance / 1.5);

  for (size_t i = 1; i * degrees_per_index < max_angle; ++i)
  {
    for (int j = -1; j < 2; j += 2)
    {
      int current_index = mod(wanted_index + (i * j), L.size());

      if (Point::isinf(L[current_index]) ||
          Point::getDistance(L[current_index]) > max_distance)
      {
        // Too far away to matter
        continue;
      }

      if (j == -1)
      {
        // To the right
        if (Point::isnan(L[current_index]))
        {
          // No reading here
          right->push_back(Point::getPointFromVectorDegrees(
              current_index * degrees_per_index, magnitude_nan));
        }
        else
        {
          right->push_back(L[current_index]);
        }
      }
      else
      {
        // To the left
        if (Point::isnan(L[current_index]))
        {
          // No reading here
          left->push_back(Point::getPointFromVectorDegrees(
              current_index * degrees_per_index, magnitude_nan));
        }
        else
        {
          left->push_back(L[current_index]);
        }
      }
    }
  }
}

double ORM::getLeftBound(const std::vector<Point>& L, double goal_direction,
                         double radius, double security_distance)
{
  double bound;
  bool first_bound = true;

  for (size_t i = 0; i < L.size(); ++i)
  {
    double obstacle_direction = Point::getDirectionDegrees(L[i]);
    double obstacle_distance = Point::getDistance(L[i]);

    double alpha = radiansToDegrees(
        std::abs(std::atan((radius + security_distance) / obstacle_distance)));

    double beta = 0;
    if (obstacle_distance <= radius + security_distance)
    {
      beta = (180.0 - alpha) *
             (1 - ((obstacle_distance - radius) / security_distance));
    }

    double temp_bound = obstacle_direction + (alpha + beta);

    if (temp_bound > 360)
    {
      temp_bound -= 360;
    }

    if (first_bound)
    {
      first_bound = false;
      bound = temp_bound;
    }
    else
    {
      double temp_diff = temp_bound - goal_direction;
      double bound_diff = bound - goal_direction;

      if (temp_diff > 180)
      {
        temp_diff -= 360;
      }
      if (bound_diff > 180)
      {
        bound_diff -= 360;
      }
      if (temp_diff < -180)
      {
        temp_diff += 360;
      }
      if (bound_diff < -180)
      {
        bound_diff += 360;
      }

      if (temp_diff > bound_diff)
      {
        bound = temp_bound;
      }
    }
  }

  return bound;
}

double ORM::getRightBound(const std::vector<Point>& L, double goal_direction,
                          double radius, double security_distance)
{
  double bound;
  bool first_bound = true;

  for (size_t i = 0; i < L.size(); ++i)
  {
    double obstacle_direction = Point::getDirectionDegrees(L[i]);
    double obstacle_distance = Point::getDistance(L[i]);

    double alpha = radiansToDegrees(
        std::abs(std::atan((radius + security_distance) / obstacle_distance)));

    double beta = 0;
    if (obstacle_distance <= radius + security_distance)
    {
      beta = (180.0 - alpha) *
             (1 - ((obstacle_distance - radius) / security_distance));
    }

    double temp_bound = obstacle_direction - (alpha + beta);

    if (temp_bound < 0)
    {
      temp_bound += 360;
    }

    if (first_bound)
    {
      first_bound = false;
      bound = temp_bound;
    }
    else
    {
      double temp_diff = temp_bound - goal_direction;
      double bound_diff = bound - goal_direction;

      if (temp_diff > 180)
      {
        temp_diff -= 360;
      }
      if (bound_diff > 180)
      {
        bound_diff -= 360;
      }
      if (temp_diff < -180)
      {
        temp_diff += 360;
      }
      if (bound_diff < -180)
      {
        bound_diff += 360;
      }

      if (temp_diff < bound_diff)
      {
        bound = temp_bound;
      }
    }
  }

  return bound;
}
}
