#include <collision_avoidance/no_input.h>
#include <collision_avoidance/obstacle_restriction_method.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace collision_avoidance
{
int mod(int a, int b)
{
  return ((a %= b) < 0) ? a + b : a;
}

ORM::ORM(double radius, double security_distance, double epsilon)
  : radius_(radius), security_distance_(security_distance), epsilon_(epsilon)
{
}

Eigen::Vector2d ORM::avoidCollision(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles)
{
  Eigen::Vector2d subgoal(subgoalSelector(goal, obstacles));

  return motionComputation(subgoal, obstacles);
}

Eigen::Vector2d ORM::subgoalSelector(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles)
{
  if (isPathClear(goal, obstacles))
  {
    // We can go straight towards goal
    return goal;
  }

  double rad_per_index = 2.0 * M_PI / obstacles.size();

  int wanted_index = std::atan2(goal[1], goal[0]) / rad_per_index;

  Eigen::Vector2d subgoal(goal);
  // We have to find a subgoal
  for (int i = 1; i < obstacles.size() / 2; ++i)
  {
    for (int j : { -1, 1 })
    {
      int current_index = mod(wanted_index + (j * i), obstacles.size());
      int previous_index = mod(wanted_index + (j * (i - 1)), obstacles.size());

      if (isSubgoal(obstacles, current_index, previous_index, &subgoal))
      {
        return subgoal;
      }
    }
  }

  return goal;
}

bool ORM::isSubgoal(const std::vector<Eigen::Vector2d>& obstacles, int index_1, int index_2, Eigen::Vector2d* subgoal)
{
  // TODO: Maybe check for NaN? Or assume that it is never NaN?
  if (!obstacles[index_1].allFinite() && !obstacles[index_2].allFinite())
  {
    // Not a subgoal becuase it is only open space
    return false;
  }

  double rad_per_index = 2.0 * M_PI / obstacles.size();

  if (!obstacles[index_1].allFinite())
  {
    // This makes it so index_1 is always finite
    int temp = index_1;
    index_1 = index_2;
    index_2 = temp;
  }

  Eigen::Vector2d temp_subgoal;

  if (!obstacles[index_2].allFinite())
  {
    // Found subgoal at the edge of an obstacle
    double distance = obstacles[index_1].norm() + radius_ + epsilon_;
    double direction = std::atan2(obstacles[index_1][1], obstacles[index_1][0]);

    temp_subgoal[0] = distance * std::cos(direction);
    temp_subgoal[1] = distance * std::sin(direction);
  }
  else
  {
    // We know that both are finite now
    if ((obstacles[index_2] - obstacles[index_1]).norm() > 2.0 * radius_)
    {
      // Found a subgoal between two obstacles
      temp_subgoal = (obstacles[index_1] + obstacles[index_2]) / 2.0;
    }
  }

  if (isPathClear(temp_subgoal, obstacles))
  {
    *subgoal = temp_subgoal;
    return true;
  }
  return false;
}

Eigen::Vector2d ORM::motionComputation(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles)
{
  std::vector<Eigen::Vector2d> left, right;

  getPointsOfInterest(goal, obstacles, &left, &right);

  double goal_direction = std::atan2(goal[1], goal[0]);

  std::pair<double, double> bounds = getBounds(goal, obstacles);

  double diff_left = radBounded(bounds.first - goal_direction);
  double diff_right = radBounded(bounds.second - goal_direction);

  if (diff_left < 0 && diff_right > 0)
  {
    // Move Straight towards goal
    return goal;
  }

  double distance = goal.norm();
  double direction;

  if (diff_right > 0 && diff_right > diff_left)
  {
    // Move towards left bound
    direction = bounds.first;
  }
  else if (diff_left < 0 && diff_right > diff_left)
  {
    // Move towards right bound
    direction = bounds.second;
  }
  else
  {
    // Move to the middle of left and right bound
    Eigen::Vector2d left_bound(std::cos(bounds.first), std::sin(bounds.first));
    Eigen::Vector2d right_bound(std::cos(bounds.second), std::sin(bounds.second));
    Eigen::Vector2d middle = (left_bound + right_bound) / 2.0;
    direction = std::atan2(middle[1], middle[0]);
  }

  return distance * Eigen::Vector2d(std::cos(direction), std::sin(direction));
}

std::pair<double, double> ORM::getBounds(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles)
{
  double max_distance = goal.norm() + radius_;  // TODO: + epsilon_ also?

  double goal_direction = std::atan2(goal[1], goal[0]);

  double left_bound = radBounded(goal_direction - (M_PI - 0.001));
  double right_bound = radBounded(goal_direction + (M_PI - 0.001));

  for (const Eigen::Vector2d& obstacle : obstacles)
  {
    if (!obstacle.allFinite())
    {
      continue;
    }

    double obstacle_distance = obstacle.norm();
    if (obstacle_distance > max_distance)
    {
      continue;
    }

    double alpha = std::fabs(std::atan((radius_ + security_distance_) / obstacle_distance));  // TODO: atan or atan2?

    double beta = 0;
    if (obstacle_distance < radius_ + security_distance_)
    {
      beta = (M_PI - alpha) * (1.0 - ((obstacle_distance - radius_) / security_distance_));
    }

    double alpha_beta = alpha + beta;

    double obstacle_direction = std::atan2(obstacle[1], obstacle[0]);
    double obstacle_direction_goal_origin = radBounded(obstacle_direction - goal_direction);
    if (obstacle_direction_goal_origin > 0)
    {
      if (obstacle_direction_goal_origin - alpha_beta < radBounded(right_bound - goal_direction))
      {
        right_bound = obstacle_direction - alpha_beta;
      }
    }
    else
    {
      if (obstacle_direction_goal_origin + alpha_beta > radBounded(left_bound - goal_direction))
      {
        left_bound = obstacle_direction + alpha_beta;
      }
    }
  }

  return std::make_pair(left_bound, right_bound);
}

bool ORM::isPathClear(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles)
{
  // A contains points on the left side of goal
  // B contains points on the right side of goal
  std::vector<Eigen::Vector2d> A, B;

  getPointsOfInterest(goal, obstacles, &A, &B);

  // Corners of the rectangle (tunnel)
  std::vector<Eigen::Vector2d> rectangle = getRectangle(goal);

  A = getPointsInPolygon(rectangle, A);
  B = getPointsInPolygon(rectangle, B);

  // Check if path is clear
  double squared_diameter = std::pow(2.0 * radius_, 2.0);
  for (const Eigen::Vector2d& a : A)
  {
    for (const Eigen::Vector2d& b : B)
    {
      if ((b - a).squaredNorm() < squared_diameter)
      {
        return false;
      }
    }
  }

  return true;
}

void ORM::getPointsOfInterest(const Eigen::Vector2d& goal, const std::vector<Eigen::Vector2d>& obstacles,
                              std::vector<Eigen::Vector2d>* left, std::vector<Eigen::Vector2d>* right)
{
  double rad_per_index = 2.0 * M_PI / obstacles.size();

  int wanted_index = std::atan2(goal[1], goal[0]) / rad_per_index;

  double max_distance = goal.norm() + radius_;

  for (size_t i = 1; i < obstacles.size() / 2; ++i)
  {
    for (int j : { -1, 1 })
    {
      int current_index = mod(wanted_index + (i * j), obstacles.size());

      if (!obstacles[current_index].allFinite() || obstacles[current_index].norm() > max_distance)
      {
        continue;
      }

      if (-1 == j)
      {
        right->emplace_back(obstacles[current_index]);
      }
      else
      {
        left->emplace_back(obstacles[current_index]);
      }
    }
  }
}

std::vector<Eigen::Vector2d> ORM::getPointsInPolygon(const std::vector<Eigen::Vector2d>& polygon,
                                                     const std::vector<Eigen::Vector2d> points)
{
  std::vector<Eigen::Vector2d> points_in_polygon;

  for (const Eigen::Vector2d& point : points)
  {
    if (!point.allFinite())
    {
      // Not possible to be inside the polygon
      continue;
    }

    if (isPointInPolygon(polygon, point))
    {
      points_in_polygon.emplace_back(point);
    }
  }

  return points_in_polygon;
}

// Source:
// https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
bool ORM::isPointInPolygon(const std::vector<Eigen::Vector2d>& polygon, const Eigen::Vector2d& point)
{
  bool c = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
  {
    if (((polygon[i][1] > point[1]) != (polygon[j][1] > point[1])) &&
        (point[0] < (polygon[j][0] - polygon[i][0]) * (point[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) +
                        polygon[i][0]))
    {
      c = !c;
    }
  }

  return c;
}

std::vector<Eigen::Vector2d> ORM::getRectangle(const Eigen::Vector2d& goal)
{
  double distance = goal.norm();

  double dx = goal[1] / distance;
  double dy = -goal[0] / distance;

  // 2 * radius becuase then all obstacles that matters will be inside the rectangle
  double diameter = 2.0 * radius_;

  std::vector<Eigen::Vector2d> rectangle;
  rectangle.emplace_back(diameter * dx, diameter * dy);
  rectangle.emplace_back(goal[0] + rectangle[0][0], goal[1] + rectangle[0][1]);
  rectangle.emplace_back(goal[0] - rectangle[0][0], goal[1] - rectangle[0][1]);
  rectangle.emplace_back(-rectangle[0][0], -rectangle[0][1]);

  return rectangle;
}
}  // namespace collision_avoidance
