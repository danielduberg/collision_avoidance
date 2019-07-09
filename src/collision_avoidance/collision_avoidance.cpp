#include <collision_avoidance/collision_avoidance.h>
#include <collision_avoidance/no_input.h>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/range_image/range_image.h>

namespace collision_avoidance
{
CollisionAvoidance::CollisionAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : map_sub_(nh.subscribe("map", 10, &CollisionAvoidance::mapCallback, this))
  , sensor_sub_(nh.subscribe("sensor", 10, &CollisionAvoidance::sensorCallback, this))
  , odometry_sub_(nh.subscribe("odometry", 10, &CollisionAvoidance::odometryCallback, this))
  , imu_sub_(nh.subscribe("imu", 10, &CollisionAvoidance::imuCallback, this))
  , control_pub_(nh_priv.advertise<geometry_msgs::TwistStamped>("control", 10))
  , path_pub_(nh_priv.advertise<nav_msgs::Path>("path", 10))
  , obstacle_pub_(nh_priv.advertise<pcl::PointCloud<pcl::PointXYZ>>("obstacles", 10))
  , as_(nh, "move_to", boost::bind(&CollisionAvoidance::goalCallback, this, _1), false)
  , tf_listener_(tf_buffer_)
  , cs_(nh_priv)
  , orm_(nh_priv)
  , robot_frame_id_(nh_priv.param<std::string>("robot_frame_id", "base_link"))
  , publish_timer_(nh_priv.createTimer(ros::Rate(nh_priv.param("frequency", 20.0)), &CollisionAvoidance::timerCallback,
                                       this, false, false))
{
  // Set up dynamic reconfigure server
  f_ = boost::bind(&CollisionAvoidance::configCallback, this, _1, _2);
  cs_.setCallback(f_);

  as_.start();
}

void CollisionAvoidance::goalCallback(const collision_avoidance::PathControlGoal::ConstPtr& goal)
{
  if (goal->path.poses.empty())
  {
    as_.setSucceeded();
    return;
  }

  if (!map_ || !last_sensor_data_ || !odometry_)
  {
    as_.setAborted();
    return;
  }

  publish_timer_.stop();

  collision_avoidance::PathControlFeedback feedback;

  nav_msgs::Path path = goal->path;
  while (!path.poses.empty())
  {
    if (as_.isPreemptRequested() || !ros::ok())
    {
      as_.setPreempted();
      publish_timer_.start();
      return;
    }

    // TODO: Maybe allow it to take shortcuts?
    // If it is possible to move to i+1, i+2, ... right away
    // maybe that is better to do then?
    // If it has taken a shortcut and gets stuck,
    // it should move back and turn off the shortcut flag
    // to resume the original path.

    target_ = path.poses.front();
    if (path.poses.size() > 1)
    {
      // Makes it so we look towards next setpoint
      double dir = std::atan2(path.poses[1].pose.position.y - target_.pose.position.y,
                              path.poses[1].pose.position.x - target_.pose.position.x);
      tf2::Quaternion q;
      q.setRPY(0, 0, dir);
      tf2::convert(q, target_.pose.orientation);
    }

    std::pair<double, double> distance;
    ros::Rate r(frequency_);
    int times_backwards = 0;
    do
    {
      target_.header.stamp = ros::Time::now();

      times_backwards = avoidCollision(target_) ? 0 : times_backwards + 1;

      if (times_backwards > max_times_backwards_)
      {
        // We cannot move towards target because it is blocked
        path.poses.clear();
        path_pub_.publish(path);
        as_.setAborted();
        target_.header = odometry_->header;
        target_.pose = odometry_->pose.pose;
        publish_timer_.start();
        return;
      }

      distance = getDistanceToTarget(target_);
      feedback.targets_left = path.poses.size();
      feedback.distance_to_next_target = distance.first;
      as_.publishFeedback(feedback);

      // Publish remaining path
      nav_msgs::Path temp_path = path;
      temp_path.header.stamp = ros::Time::now();
      if (odometry_)
      {
        geometry_msgs::PoseStamped pose;
        pose.header = temp_path.header;
        pose.pose = odometry_->pose.pose;
        temp_path.poses.insert(temp_path.poses.begin(), pose);
      }
      path_pub_.publish(temp_path);

      r.sleep();
    } while (distance.first > distance_converged_);  // || distance.second > yaw_converged_);

    path.poses.erase(path.poses.begin());
  }

  as_.setSucceeded();
  publish_timer_.start();
}

bool CollisionAvoidance::avoidCollision(geometry_msgs::PoseStamped setpoint)
{
  try
  {
    geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(robot_frame_id_, setpoint.header.frame_id, ros::Time(0));
    tf2::doTransform(setpoint, setpoint, transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "Avoid collision: %s", ex.what());
    noInput(setpoint);
    return true;  // TODO: What should it return?
  }

  Eigen::Vector2d goal(setpoint.pose.position.x, setpoint.pose.position.y);
  Eigen::Vector2d control_2d;

  PolarHistogram obstacles = getObstacles(goal.norm() + (2.0 * radius_) + min_distance_hold_);
  publishObstacles(obstacles);

  double yaw = 0;
  if (goal.norm() > distance_converged_)
  {
    yaw = std::atan2(goal[1], goal[0]);
  }
  else
  {
    yaw = tf2::getYaw(setpoint.pose.orientation);
  }

  bool valid = true;

  if (std::abs(yaw) < yaw_converged_)
  {
    // Is this correct?
    goal[0] -= odometry_->twist.twist.linear.x;
    goal[1] -= odometry_->twist.twist.linear.y;

    control_2d = orm_.avoidCollision(goal, obstacles, robot_frame_id_);

    double direction_change =
        std::fabs(std::remainder(std::atan2(goal[1], goal[0]) - std::atan2(control_2d[1], control_2d[0]), 2 * M_PI));

    if (control_2d.isZero() || direction_change > max_direction_change_)
    {
      control_2d = no_input::avoidCollision(Eigen::Vector2d(0, 0), obstacles, radius_, min_distance_hold_);
      valid = false;
    }
    // else
    // {
    //   if (goal.norm() > distance_converged_)
    //   {
    //     yaw = std::atan2(control_2d[1], control_2d[0]);
    //   }
    // }

    ROS_INFO("Direction change: %.2f (deg)", direction_change * 180.0 / M_PI);
  }
  else
  {
    control_2d = no_input::avoidCollision(Eigen::Vector2d(0, 0), obstacles, radius_, min_distance_hold_);
  }

  double direction = std::atan2(control_2d[1], control_2d[0]);
  double magnitude = std::clamp(control_2d.norm(), -max_xy_vel_, max_xy_vel_);

  geometry_msgs::TwistStamped control;
  control.header.frame_id = robot_frame_id_;
  control.header.stamp = ros::Time::now();
  control.twist.linear.x = magnitude * std::cos(direction);
  control.twist.linear.y = magnitude * std::sin(direction);
  control.twist.linear.z = std::clamp(setpoint.pose.position.z, -max_z_vel_, max_z_vel_);
  control.twist.angular.z = std::clamp(yaw, -max_yaw_rate_, max_yaw_rate_);

  adjustVelocity(&control, obstacles);
  control_pub_.publish(control);

  return valid;
}

void CollisionAvoidance::noInput(geometry_msgs::PoseStamped setpoint) const
{
  try
  {
    geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(robot_frame_id_, setpoint.header.frame_id, ros::Time(0));
    tf2::doTransform(setpoint, setpoint, transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "No input: %s", ex.what());
    return;
  }

  Eigen::Vector2d goal(setpoint.pose.position.x, setpoint.pose.position.y);

  PolarHistogram obstacles = getObstacles(goal.norm() + (2.0 * radius_) + min_distance_hold_);
  publishObstacles(obstacles);

  Eigen::Vector2d control_2d(no_input::avoidCollision(Eigen::Vector2d(0, 0), obstacles, radius_, min_distance_hold_));
  double direction = std::atan2(control_2d[1], control_2d[0]);
  double magnitude = std::clamp(control_2d.norm(), -max_xy_vel_, max_xy_vel_);

  geometry_msgs::TwistStamped control;
  control.header.frame_id = robot_frame_id_;
  control.header.stamp = ros::Time::now();
  control.twist.linear.x = magnitude * std::cos(direction);
  control.twist.linear.y = magnitude * std::sin(direction);
  control.twist.linear.z = std::clamp(setpoint.pose.position.z, -max_z_vel_, max_z_vel_);
  control.twist.angular.z = std::clamp(tf2::getYaw(setpoint.pose.orientation), -max_yaw_rate_, max_yaw_rate_);

  adjustVelocity(&control, obstacles);
  control_pub_.publish(control);
}

void CollisionAvoidance::publishObstacles(const PolarHistogram& obstacles) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->header.frame_id = robot_frame_id_;
  pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
  cloud->width = obstacles.numBuckets();
  cloud->height = 1;
  for (const PolarHistogram::Vector& obstacle : obstacles)
  {
    Eigen::Vector2d p = obstacle.getPoint();
    cloud->push_back(pcl::PointXYZ(p[0], p[1], 0));
  }
  obstacle_pub_.publish(cloud);
}

void CollisionAvoidance::adjustVelocity(geometry_msgs::TwistStamped* control, const PolarHistogram& obstacles) const
{
  double closest_obstacle_distance = std::numeric_limits<double>::infinity();
  for (const PolarHistogram::Vector& obstacle : obstacles)
  {
    closest_obstacle_distance = std::min(closest_obstacle_distance, obstacle.getRange());
  }

  Eigen::Vector2d goal(control->twist.linear.x, control->twist.linear.y);
  double direction = std::atan2(goal[1], goal[0]);
  double updated_magnitude = max_xy_vel_ * std::min(closest_obstacle_distance / (h_m_ * radius_), goal.norm());

  control->twist.linear.x = updated_magnitude * std::cos(direction);
  control->twist.linear.y = updated_magnitude * std::sin(direction);
}

PolarHistogram CollisionAvoidance::getObstacles(double obstacle_window) const
{
  PolarHistogram obstacles(num_histogram_, std::numeric_limits<double>::infinity());

  for (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ptr : { map_, last_sensor_data_ })
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_ptr, *cloud, indices);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    sor.filter(*cloud);

    geometry_msgs::TransformStamped tf_transform;
    try
    {
      tf_transform = tf_buffer_.lookupTransform(cloud_ptr->header.frame_id, robot_frame_id_, ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1, "Get obstacles: %s", ex.what());
      continue;  // TODO: Fix
    }

    double yaw, pitch, roll;
    tf2::getEulerYPR(tf_transform.transform.rotation, yaw, pitch, roll);

    pcl::CropBox<pcl::PointXYZ> box_filter;
    box_filter.setMin(Eigen::Vector4f(-obstacle_window, -obstacle_window, -(height_ / 2.0), 0.0));
    box_filter.setMax(Eigen::Vector4f(obstacle_window, obstacle_window, height_ / 2.0, 1.0));
    box_filter.setTranslation(Eigen::Vector3f(tf_transform.transform.translation.x,
                                              tf_transform.transform.translation.y,
                                              tf_transform.transform.translation.z));
    box_filter.setRotation(Eigen::Vector3f(roll, pitch, yaw));
    box_filter.setInputCloud(cloud);
    box_filter.filter(*cloud);

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << tf_transform.transform.translation.x, tf_transform.transform.translation.y,
        tf_transform.transform.translation.z;
    transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
    transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud, transform.inverse());

    for (const pcl::PointXYZ& point : *cloud)
    {
      if (!pcl::isFinite(point))
      {
        continue;
      }

      pcl::PointXYZ p = point;
      p.x = point.x - odometry_->twist.twist.linear.x;
      p.y = point.y - odometry_->twist.twist.linear.y;

      double direction = std::atan2(p.y, p.x);
      double range = std::hypot(p.x, p.y);
      if (!obstacles.isFinite(direction) || range < obstacles.getRange(direction))
      {
        obstacles.setRange(direction, range);
      }
    }
  }

  return obstacles;
}

std::pair<double, double> CollisionAvoidance::getDistanceToTarget(const geometry_msgs::PoseStamped& target)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform =
        tf_buffer_.lookupTransform(target.header.frame_id, robot_frame_id_, ros::Time::now(), ros::Duration(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "Get distance to target: %s", ex.what());
    return std::make_pair(std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity());  // TODO: Fix
  }

  Eigen::Vector3d current_position(transform.transform.translation.x, transform.transform.translation.y,
                                   transform.transform.translation.z);
  Eigen::Vector3d target_position(target.pose.position.x, target.pose.position.y, target.pose.position.z);
  double distance = (target_position - current_position).norm();

  double current_yaw = tf2::getYaw(transform.transform.rotation);
  double target_yaw = tf2::getYaw(target.pose.orientation);
  double yaw_diff = std::fabs(std::atan2(std::sin(current_yaw - target_yaw), std::cos(current_yaw - target_yaw)));

  return std::pair(distance, yaw_diff);
}

void CollisionAvoidance::timerCallback(const ros::TimerEvent& event)
{
  noInput(target_);
}

void CollisionAvoidance::configCallback(const collision_avoidance::CollisionAvoidanceConfig& config, uint32_t level)
{
  distance_converged_ = config.distance_converged;
  yaw_converged_ = config.yaw_converged;

  if (frequency_ != config.frequency)
  {
    frequency_ = config.frequency;
    publish_timer_.setPeriod(ros::Duration(1.0 / frequency_), false);
  }

  max_xy_vel_ = config.max_xy_vel;
  max_z_vel_ = config.max_z_vel;
  max_yaw_rate_ = config.max_yaw_rate * M_PI / 180.0;
  h_m_ = config.h_m;

  radius_ = config.radius;
  height_ = config.height;
  min_distance_hold_ = config.min_distance_hold;

  num_histogram_ = config.polar_size;

  max_times_backwards_ = config.max_times_backwards;
  max_direction_change_ = config.max_direction_change * M_PI / 180.0;

  leaf_size_ = config.leaf_size;

  orm_.setRadius(radius_);
  orm_.setSecurityDistance(config.security_distance);
  orm_.setEpsilon(config.epsilon);
}
}  // namespace collision_avoidance
