#include <collision_avoidance/collision_avoidance.h>

#include <tf2/utils.h>

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>

namespace collision_avoidance
{
CollisionAvoidance::CollisionAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : map_sub_(nh.subscribe("map", 10, &CollisionAvoidance::mapCallback, this))
  , sensor_sub_(nh.subscribe("sensor", 10, &CollisionAvoidance::sensorCallback, this))
  , odometry_sub_(nh.subscribe("odometry", 10, &CollisionAvoidance::odometryCallback, this))
  , imu_sub_(nh.subscribe("imu", 10, &CollisionAvoidance::imuCallback, this))
  , control_pub_(nh_priv.advertise<geometry_msgs::PoseStamped>("control", 10))

  , as_(nh, "move_to", boost::bind(&CollisionAvoidance::goalCallback, this, _1), false)
  , tf_listener_(tf_buffer_)
  , cs_(nh_priv)
  , robot_frame_id_(nh_priv.param<std::string>("robot_frame_id", "base_link"))
  , publish_timer_(nh_priv.createTimer(ros::Rate(frequency_), &CollisionAvoidance::timerCallback, this, false, false))
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
    as_.setAborted();
    return;
  }

  publish_timer_.stop();

  collision_avoidance::PathControlFeedback feedback;

  for (size_t i = 0; i < goal->path.poses.size(); ++i)
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

    target_ = goal->path.poses[i];

    std::pair<double, double> distance;
    ros::Rate r(frequency_);
    int times_backwards = 0;
    do
    {
      times_backwards = avoidCollision(target_) ? 0 : times_backwards + 1;

      if (times_backwards > max_times_backwards_)
      {
        // We cannot move towards target because it is blocked
        as_.setAborted();
        target_.header = odometry_->header;
        target_.pose = odometry_->pose.pose;
        publish_timer_.start();
        return;
      }

      distance = getDistanceToTarget(target_);
      feedback.targets_left = goal->path.poses.size() - i;
      feedback.distance_to_next_target = distance.first;
      as_.publishFeedback(feedback);
      r.sleep();
    } while (distance.first > distance_converged_ || distance.second > yaw_converged_);
  }

  as_.setSucceeded();
  publish_timer_.start();
}

bool CollisionAvoidance::avoidCollision(geometry_msgs::PoseStamped setpoint)
{
  try
  {
    setpoint = tf_buffer_.transform(setpoint, robot_frame_id_);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "%s", ex.what());

    // TODO: No input
    return true;  // TODO: What should it return?
  }

  geometry_msgs::TwistStamped control;
  control.header = setpoint.header;
  control.twist.angular.z = tf2::getYaw(setpoint.pose.orientation);

  if (control.twist.angular.z < yaw_converged_)
  {
    Eigen::Vector2d goal(setpoint.pose.position.x, setpoint.pose.position.y);

    PolarHistogram obstacles = getObstacles();

    Eigen::Vector2d control_2d(orm_.avoidCollision(goal, obstacles));

    double direction_change =
        std::fabs(std::remainder(std::atan2(goal[1], goal[0]) - std::atan2(control_2d[1], control_2d[0]), 2 * M_PI));

    control.twist.linear.x = control_2d[0];
    control.twist.linear.y = control_2d[1];
    control.twist.linear.z = std::clamp(setpoint.pose.position.z, -max_z_vel_, max_z_vel_);
    control.twist.angular.z = std::clamp(control.twist.angular.z, -max_yaw_rate_, max_yaw_rate_);
    control_pub_.publish(control);

    return direction_change <= max_direction_change_;
  }
  else
  {
    // TODO: No input?
  }

  return true;
}

PolarHistogram CollisionAvoidance::getObstacles()
{
  // Get data from map

  // TODO: Try-catch?
  geometry_msgs::TransformStamped tf_transform =
      tf_buffer_.lookupTransform(map_->header.frame_id, robot_frame_id_, ros::Time::now());  // Correct?

  pcl::RangeImage range_image;
  Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
  sensor_pose.translation() << tf_transform.transform.translation.x, tf_transform.transform.translation.y,
      tf_transform.transform.translation.z;
  double yaw, pitch, roll;
  tf2::getEulerYPR(tf_transform.transform.rotation, yaw, pitch, roll);
  sensor_pose.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  sensor_pose.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
  sensor_pose.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

  range_image.createFromPointCloud(*map_, pcl::deg2rad(1.0), pcl::deg2rad(360.0), pcl::deg2rad(180.0), sensor_pose,
                                   pcl::RangeImage::LASER_FRAME, 0, 0, 1);

  PolarHistogram obstacles(num_histogram_, std::numeric_limits<double>::infinity());

  double rad_per_index = 2.0 * M_PI / obstacles.numBuckets();
  for (const pcl::PointWithRange& pr : range_image)
  {
    if (!pcl::isFinite(pr))
    {
      continue;
    }

    double direction = std::atan2(pr.y, pr.x);
    if (!obstacles.isFinite(direction) || pr.range < obstacles.getRange(direction))
    {
      obstacles.setRange(direction, pr.range);
    }
  }

  // Get data from latest sensor reading
  tf_transform =
      tf_buffer_.lookupTransform(last_sensor_data_->header.frame_id, robot_frame_id_, ros::Time::now());  // Correct?

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << tf_transform.transform.translation.x, tf_transform.transform.translation.y,
      tf_transform.transform.translation.z;
  tf2::getEulerYPR(tf_transform.transform.rotation, yaw, pitch, roll);
  transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sensor_data(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*last_sensor_data_, *transformed_sensor_data, transform);

  for (const pcl::PointXYZ& p : *transformed_sensor_data)
  {
    if (!pcl::isFinite(p))
    {
      continue;
    }

    double direction = std::atan2(p.x, p.y);
    double range = std::hypot(p.x, p.y);
    if (!obstacles.isFinite(direction) || range < obstacles.getRange(direction))
    {
      obstacles.setRange(direction, range);
    }
  }

  return obstacles;
}

std::pair<double, double> CollisionAvoidance::getDistanceToTarget(const geometry_msgs::PoseStamped& target)
{
  geometry_msgs::TransformStamped transform =
      tf_buffer_.lookupTransform(target.header.frame_id, robot_frame_id_, ros::Time::now());

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
  target_.header.stamp = ros::Time::now();
  avoidCollision(target_);
}

void CollisionAvoidance::configCallback(const collision_avoidance::CollisionAvoidanceConfig& config, uint32_t level)
{
  distance_converged_ = config.distance_converged;
  yaw_converged_ = config.yaw_converged;

  frequency_ = config.frequency;

  max_xy_vel_ = config.max_xy_vel;
  max_z_vel_ = config.max_z_vel;
  max_yaw_rate_ = config.max_yaw_rate * M_PI / 180.0;

  radius_ = config.radius;
  height_ = config.height;
  min_distance_hold_ = config.min_distance_hold;

  num_histogram_ = config.polar_size;

  orm_.setRadius(radius_);
  orm_.setSecurityDistance(config.security_distance);
  orm_.setEpsilon(config.epsilon);
}
}  // namespace collision_avoidance
