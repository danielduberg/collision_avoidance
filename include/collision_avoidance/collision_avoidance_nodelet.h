#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <collision_avoidance/point.h>
#include <collision_avoidance/obstacle_restriction_method.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <dynamic_reconfigure/server.h>
#include <collision_avoidance/CollisionAvoidanceConfig.h>

namespace collision_avoidance
{
class CANodelet : public nodelet::Nodelet
{

private:
  // Obstacles
  std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacle_cloud_;

  // Polar Histogram
  double max_data_age_;
  int polar_size_;

  // Current pose
  geometry_msgs::PoseStamped current_pose_;

  // Current imu;
  sensor_msgs::Imu imu_;

  // Current velocity
  double current_x_vel_;
  double current_y_vel_;

  // Obstacle-Restriction Method
  double radius_;
  double security_distance_;
  double epsilon_;

  // Robot
  std::string robot_frame_;
  double robot_height_;

  // Maximum velocities
  double max_xy_vel_;
  double max_z_vel_;
  double max_yaw_rate_;

  // Tele-op
  double min_change_in_direction_;
  double max_change_in_direction_;
  double min_opposite_direction_;
  double max_opposite_direction_;

  // Ego-Dynamic Space
  double ab_;
  double T_;

  // No input
  double min_distance_hold_;

  // Subscribers
  std::vector<ros::Subscriber> obstacle_sub_;
  ros::Subscriber setpoint_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber imu_sub_;

  // Publishers
  ros::Publisher collision_free_control_pub_;
  ros::Publisher cloud_before_pub_;
  ros::Publisher cloud_after_pub_;
  ros::Publisher cloud_obstacle_pub_;

  ros::Timer no_input_timer_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig>
      cs_;
  dynamic_reconfigure::Server<
      collision_avoidance::CollisionAvoidanceConfig>::CallbackType f_;

  // Transform
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;

private:
  virtual void onInit();

  void transformPointcloud(const std::string& target_frame,
                           const std::string& source_frame,
                           const sensor_msgs::PointCloud2::ConstPtr& cloud_in,
                           sensor_msgs::PointCloud2* cloud_out);

  void rosToPcl(const sensor_msgs::PointCloud2& cloud_in,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                          int index);

  std::vector<Point> getPolarHistogram(
      const std::vector<pcl::PointCloud<pcl::PointXYZ>>& obstacle_cloud);

  void setpointCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);

  void timerCallback(const ros::TimerEvent& event);

  std::vector<Point> getEgeDynamicSpace(const std::vector<Point>& obstacles_in);

  void adjustVelocity(const std::vector<Point>& obstacles,
                      geometry_msgs::TwistStamped* control,
                      const double magnitude);

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  void configCallback(collision_avoidance::CollisionAvoidanceConfig& config,
                      uint32_t level);
};
}
