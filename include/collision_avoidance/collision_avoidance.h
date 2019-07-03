#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <collision_avoidance/PathControlAction.h>

#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <collision_avoidance/obstacle_restriction_method.h>

#include <collision_avoidance/CollisionAvoidanceConfig.h>
#include <dynamic_reconfigure/server.h>

namespace collision_avoidance
{
class CollisionAvoidance
{
private:
  // Subscribers
  ros::Subscriber map_sub_;
  ros::Subscriber sensor_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber imu_sub_;

  // Publishers
  ros::Publisher control_pub_;

  // Action servers
  actionlib::SimpleActionServer<collision_avoidance::PathControlAction> as_;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig> cs_;
  dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig>::CallbackType f_;

  // Stored data
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr map_;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr last_sensor_data_;
  nav_msgs::Odometry::ConstPtr odometry_;
  sensor_msgs::Imu::ConstPtr imu_;

  // Current target
  geometry_msgs::PoseStamped target_;

  // ORM
  ORM orm_;

  // Robot frame_id
  std::string robot_frame_id_;

  // Converge criterion
  double distance_converged_;
  double yaw_converged_;

  // Publish frequency
  double frequency_;

  // Timers
  ros::Timer publish_timer_;

  // Max allowed velocity
  double max_xy_vel_;
  double max_z_vel_;
  double max_yaw_rate_;

  // 
  double max_direction_change_;
  int max_times_backwards_;

  int num_histogram_;

public:
  CollisionAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void goalCallback(const collision_avoidance::PathControlGoal::ConstPtr& goal);

  bool avoidCollision(geometry_msgs::PoseStamped setpoint);

  std::vector<Eigen::Vector2d> getObstacles();

  std::pair<double, double> getDistanceToTarget(const geometry_msgs::PoseStamped& target);

  void timerCallback(const ros::TimerEvent& event);

  void mapCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map)
  {
    map_ = map;
  }
  void sensorCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& data)
  {
    last_sensor_data_ = data;
  }
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometry)
  {
    odometry_ = odometry;
  }
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
  {
    imu_ = imu;
  }

  void configCallback(const collision_avoidance::CollisionAvoidanceConfig& config, uint32_t level);
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_H