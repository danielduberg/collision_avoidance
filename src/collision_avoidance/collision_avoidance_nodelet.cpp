#include <pluginlib/class_list_macros.h>

#include <collision_avoidance/collision_avoidance_nodelet.h>
#include <collision_avoidance/no_input.h>

#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <pcl/PCLPointCloud2.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(collision_avoidance::CANodelet, nodelet::Nodelet)

namespace collision_avoidance
{
void CANodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& nh_priv = getPrivateNodeHandle();

  // Set up dynamic reconfigure server
  f_ = boost::bind(&CANodelet::configCallback, this, _1, _2);
  cs_ = new dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig>(nh_priv);
  cs_->setCallback(f_);

  tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

  std::vector<std::string> obstacle_topics;
  if (!nh_priv.getParam("obstacle_topics", obstacle_topics))
  {
    NODELET_ERROR("Failed to get param 'obstacle_topics'");
  }

  obstacle_cloud_.resize(obstacle_topics.size());
  obstacle_sub_.reserve(obstacle_topics.size());

  for (int i = 0; i < obstacle_topics.size(); ++i)
  {
    obstacle_sub_.push_back(nh.subscribe<sensor_msgs::PointCloud2>(
        obstacle_topics[i], 1,
        boost::bind(&CANodelet::pointCloudCallback, this, _1, i)));
  }

  setpoint_sub_ =
      nh.subscribe("/setpoint", 1, &CANodelet::setpointCallback, this);
  odometry_sub_ = nh.subscribe<nav_msgs::Odometry>(
      "/mavros/local_position/odom", 1, &CANodelet::odometryCallback, this);
  // Change this to use odom instead?
  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(
      "/mavros/local_position/pose", 1, &CANodelet::poseCallback, this);
  imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1,
                                            &CANodelet::imuCallback, this);

  collision_free_control_pub_ =
      nh.advertise<geometry_msgs::TwistStamped>("/collision_free_control", 1);
  cloud_before_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_before_imu", 1);
  cloud_after_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_after_imu", 1);
  cloud_obstacle_pub_ =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_obstacle", 1);

  no_input_timer_ =
      nh.createTimer(ros::Rate(5), &CANodelet::timerCallback, this);
}

void CANodelet::transformPointcloud(
    const std::string& target_frame, const std::string& source_frame,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_in,
    sensor_msgs::PointCloud2* cloud_out)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform =
        tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    throw(ex);
  }

  // Source:
  // http://docs.ros.org/jade/api/tf2_sensor_msgs/html/tf2__sensor__msgs_8h_source.html
  tf2::doTransform(*cloud_in, *cloud_out, transform);
}

void CANodelet::rosToPcl(const sensor_msgs::PointCloud2& in_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
{
  pcl::PCLPointCloud2 temp_cloud;
  pcl_conversions::toPCL(in_cloud, temp_cloud);
  pcl::fromPCLPointCloud2(temp_cloud, *out_cloud);
}

void CANodelet::pointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud, int index)
{
  // Transform msg to correct frame
  sensor_msgs::PointCloud2 transformed_cloud2;
  try
  {
    transformPointcloud(robot_frame_, cloud->header.frame_id, cloud,
                        &transformed_cloud2);
  }
  catch (tf2::TransformException& ex)
  {
    NODELET_WARN("%s", ex.what());
    return;
  }

  cloud_before_pub_.publish(transformed_cloud2);

  sensor_msgs::PointCloud2 transformed_cloud;

  // Stabilize it using IMU data
  double yaw, pitch, roll;
  tf2::getEulerYPR(imu_.orientation, yaw, pitch, roll);
  tf2::Quaternion q;
  q.setRPY(roll, pitch, 0);
  geometry_msgs::TransformStamped transform;
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf2::doTransform(transformed_cloud2, transformed_cloud, transform);

  transformed_cloud.header = transformed_cloud2.header;

  cloud_after_pub_.publish(transformed_cloud);

  // Convert it to PCL for easier access to elements
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  rosToPcl(transformed_cloud, pcl_cloud);

  // Insert the new data starting from starting_index to starting_index +
  // new_data->width
  obstacle_cloud_[index].resize(pcl_cloud->width);
  obstacle_cloud_[index].header = pcl_cloud->header;

  for (size_t i = 0; i < pcl_cloud->width; ++i)
  {
    pcl::PointXYZ point;
    point.x = std::numeric_limits<float>::quiet_NaN();
    point.y = std::numeric_limits<float>::quiet_NaN();
    point.z = std::numeric_limits<float>::quiet_NaN();

    if (!pcl_cloud->is_dense)
    {
      // Take the closest point in column that the robot can collied into
      for (size_t j = 0; j < pcl_cloud->height; ++j)
      {
        if (pcl_isfinite(pcl_cloud->at(i, j).z) &&
            std::fabs(pcl_cloud->at(i, j).z) <= 0.6 * robot_height_)
        {
          // This point is at a height such that the robot can collied with it
          point.x = pcl_cloud->at(i, j).x;
          point.y = pcl_cloud->at(i, j).y;
          point.z = pcl_cloud->at(i, j).z;
        }
      }
    }
    else
    {
      if (pcl_isfinite((*pcl_cloud)[i].z) &&
          std::fabs((*pcl_cloud)[i].z) <=
              0.6 *
                  robot_height_) // && std::fabs((*pcl_cloud)[i].z) <= radius_)
      {
        // This point is at a height such that the robot can collied with it
        point.x = (*pcl_cloud)[i].x;
        point.y = (*pcl_cloud)[i].y;
        point.z = (*pcl_cloud)[i].z;
      }
    }

    obstacle_cloud_[index][i] = point;
  }

  cloud_obstacle_pub_.publish(obstacle_cloud_[index]);
}

std::vector<Point> CANodelet::getPolarHistogram(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>>& cloud)
{
  std::vector<Point> polar_hist;
  polar_hist.resize(polar_size_);

  for (size_t i = 0; i < cloud.size(); ++i)
  {
    if ((ros::Time::now() - pcl_conversions::fromPCL(cloud[i].header.stamp))
            .toSec() > max_data_age_)
    {
      // This is older than 1 second, discard!
      // ROS_FATAL("Hej from %f seconds ago", (ros::Time::now() -
      // pcl_conversions::fromPCL(cloud[i].header.stamp)).toSec());
      // continue;
    }

    int last_index = -1;

    for (size_t j = 0; j < cloud[i].size(); ++j)
    {
      Point p(cloud[i][j].x, cloud[i][j].y);

      if (!Point::isfinite(p))
      {
        continue;
      }

      int index = ((int)round(Point::getDirectionDegrees(p) *
                              (polar_hist.size() / 360.0))) %
                  polar_hist.size();

      double distance = Point::getDistance(p);

      if (distance > 1000)
      {
        distance = std::numeric_limits<double>::infinity();
        p.x_ = std::numeric_limits<double>::infinity();
        p.y_ = std::numeric_limits<double>::infinity();
      }

      if (distance < radius_)
      {
        // Point is inside robot?
        continue;
      }

      if (Point::isnan(polar_hist[index]) ||
          distance < Point::getDistance(polar_hist[index]))
      {
        polar_hist[index] = p;
      }

      // Set everything in between to infinity
      if (last_index != -1)
      {
        if (std::max(last_index, index) - std::min(last_index, index) <
            polar_hist.size() / 2.0)
        {
          for (size_t k = std::min(last_index, index);
               k < std::max(last_index, index); ++k)
          {
            if (Point::isnan(polar_hist[k]))
            {
              polar_hist[k] = Point(std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity());
            }
          }
        }
        else
        {
          for (size_t k = std::max(last_index, index);
               k < polar_hist.size() + std::min(last_index, index); ++k)
          {
            size_t k_360 = k % 360;

            if (Point::isnan(polar_hist[k_360]))
            {
              polar_hist[k_360] =
                  Point(std::numeric_limits<double>::infinity(),
                        std::numeric_limits<double>::infinity());
            }
          }
        }
      }
      last_index = index;
    }
  }

  return polar_hist;
}

void CANodelet::setpointCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  no_input_timer_.stop();

  saved_pose_ = current_pose_;

  avoidCollision(*msg);

  no_input_timer_.start();
}

void CANodelet::avoidCollision(const geometry_msgs::PoseStamped& setpoint)
{
  geometry_msgs::TwistStamped collision_free_control;

  // Transform to robot frame
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
        robot_frame_, setpoint.header.frame_id, ros::Time(0));

    geometry_msgs::PoseStamped setpoint_transformed;
    tf2::doTransform(setpoint, setpoint_transformed, transform);

    collision_free_control.header = setpoint_transformed.header;
    collision_free_control.twist.linear.x =
        setpoint_transformed.pose.position.x;
    collision_free_control.twist.linear.y =
        setpoint_transformed.pose.position.y;
    collision_free_control.twist.linear.z =
        setpoint_transformed.pose.position.z;

    collision_free_control.twist.angular.z =
        tf2::getYaw(setpoint_transformed.pose.orientation);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(1, "Collision Avoidance: %s", ex.what());
    collision_free_control.header.stamp = ros::Time::now();
    collision_free_control.header.frame_id = robot_frame_;
  }

  collision_free_control.twist.linear.z =
      std::min(collision_free_control.twist.linear.z, max_z_vel_);
  collision_free_control.twist.linear.z =
      std::max(collision_free_control.twist.linear.z, -max_z_vel_);

  collision_free_control.twist.angular.z =
      std::min(collision_free_control.twist.angular.z, max_yaw_rate_);
  collision_free_control.twist.angular.z =
      std::max(collision_free_control.twist.angular.z, -max_yaw_rate_);

  double magnitude =
      std::min(std::sqrt(std::pow(collision_free_control.twist.linear.x, 2) +
                         std::pow(collision_free_control.twist.linear.y, 2)),
               1.0);

  std::vector<Point> obstacles = getPolarHistogram(obstacle_cloud_);

  obstacles = getEgeDynamicSpace(obstacles);

  // First pass
  ORM::avoidCollision(&collision_free_control, magnitude, obstacles, radius_,
                      security_distance_, epsilon_, min_distance_hold_,
                      min_change_in_direction_, max_change_in_direction_,
                      min_opposite_direction_, max_opposite_direction_);

  adjustVelocity(obstacles, &collision_free_control, magnitude);

  // Publish the collision free control
  collision_free_control_pub_.publish(collision_free_control);
}

void CANodelet::timerCallback(const ros::TimerEvent& event)
{
  saved_pose_.header.stamp = ros::Time::now();

  avoidCollision(saved_pose_);

  /*
  geometry_msgs::TwistStamped collision_free_control;
  collision_free_control.header.stamp = ros::Time::now();
  collision_free_control.header.frame_id = robot_frame_;

  std::vector<Point> obstacles = getPolarHistogram(obstacle_cloud_);

  obstacles = getEgeDynamicSpace(obstacles);

  NoInput::avoidCollision(&collision_free_control, obstacles, radius_,
                          min_distance_hold_);

  double magnitude =
      std::min(std::sqrt(std::pow(collision_free_control.twist.linear.x, 2) +
                         std::pow(collision_free_control.twist.linear.y, 2)),
               1.0);

  adjustVelocity(obstacles, &collision_free_control, magnitude);

  collision_free_control_pub_.publish(collision_free_control);
  */
}

std::vector<Point>
CANodelet::getEgeDynamicSpace(const std::vector<Point>& obstacles_in)
{
  std::vector<Point> obstacles_out;

  obstacles_out.reserve(obstacles_in.size());
  for (size_t i = 0; i < obstacles_in.size(); ++i)
  {
    if (!Point::isfinite(obstacles_in[i]))
    {
      // No reading here
      obstacles_out.push_back(obstacles_in[i]);
      continue;
    }

    //          // Decrease the distance with 5 cm?!
    //          double dobs = Point::getDistance(obstacles_in[i]);

    //          dobs -= radius_;

    //          double deff = ab_ * (T_ * T_) * (std::sqrt(1.0 + ((2.0 * dobs) /
    //          (ab_ * (T_ * T_)))) - 1.0);

    //          deff += radius_;

    //          deff = std::min(Point::getDistance(obstacles_in[i]), deff);
    //          deff = std::max(deff, radius_ + 0.01);

    //          Point p;
    //          p.x_ = deff * std::cos(Point::getDirection(obstacles_in[i]));
    //          p.y_ = deff * std::sin(Point::getDirection(obstacles_in[i]));

    Point p;
    p.x_ = obstacles_in[i].x_ - current_y_vel_;
    p.y_ = obstacles_in[i].y_ - current_x_vel_;

    if (Point::getDistance(p) < radius_)
    {
      // Move p 1 cm out of the robot
      p = Point::getPointFromVectorDegrees(
          Point::getDirectionDegrees(obstacles_in[i]), radius_ + 0.01);
    }

    // TODO: Check that it is above 0

    obstacles_out.push_back(p);
  }

  return obstacles_out;
}

void CANodelet::adjustVelocity(const std::vector<Point>& obstacles,
                               geometry_msgs::TwistStamped* control,
                               const double magnitude)
{
  if (control->twist.linear.x == 0 && control->twist.linear.y == 0)
  {
    return;
  }

  double closest_obstacle_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < obstacles.size(); ++i)
  {
    double distance = Point::getDistance(obstacles[i]);

    if (distance != 0 && distance < closest_obstacle_distance)
    {
      closest_obstacle_distance = distance;
    }
  }

  Point goal(control->twist.linear.x, control->twist.linear.y);
  double direction = Point::getDirectionDegrees(goal);

  double updated_magnitude =
      max_xy_vel_ * std::min(closest_obstacle_distance /
                                 (h_m_ * (radius_ + security_distance_)),
                             magnitude);

  Point updated_goal =
      Point::getPointFromVectorDegrees(direction, updated_magnitude);

  control->twist.linear.x = updated_goal.x_;
  control->twist.linear.y = updated_goal.y_;
}

void CANodelet::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_x_vel_ = msg->twist.twist.linear.y; // This should be y?
  current_y_vel_ = msg->twist.twist.linear.x; // This should be x?
}

void CANodelet::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose_ = *msg;
}

void CANodelet::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  imu_ = *imu;

  double yaw, pitch, roll;
  tf2::getEulerYPR(imu_.orientation, yaw, pitch, roll);

  // ROS_FATAL("%f, %f, %f", yaw, pitch, roll);
}

void CANodelet::configCallback(CollisionAvoidanceConfig& config, uint32_t level)
{
  robot_frame_ = config.robot_frame;
  robot_height_ = config.robot_height;

  radius_ = config.radius;
  security_distance_ = config.security_distance;
  epsilon_ = config.epsilon;

  min_distance_hold_ = config.min_distance_hold;
  h_m_ = config.h_m;

  min_change_in_direction_ = config.min_change_in_direction;
  max_change_in_direction_ = config.max_change_in_direction;

  min_opposite_direction_ = config.min_opposite_direction;
  max_opposite_direction_ = config.max_opposite_direction;

  max_data_age_ = config.max_data_age;
  polar_size_ = config.polar_size;

  max_xy_vel_ = config.max_xy_vel;
  max_z_vel_ = config.max_z_vel;
  max_yaw_rate_ = config.max_yaw_rate * M_PI / 180.0;

  min_xy_vel_ = config.min_xy_vel;
  min_z_vel_ = config.min_z_vel;
  min_yaw_rate_ = config.min_yaw_rate * M_PI / 180.0;
}
}
