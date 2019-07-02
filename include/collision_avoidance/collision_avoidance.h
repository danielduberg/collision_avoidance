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

public:
  CollisionAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
  void goalCallback(const collision_avoidance::PathControlGoal::ConstPtr& goal);

  bool avoidCollision(geometry_msgs::PoseStamped setpoint);

  void getObstacles(std::vector<Eigen::Vector2d>* obstacles);

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

  //   ros::Subscriber obstacle_sub_;
  //   ros::Subscriber setpoint_sub_;
  //   ros::Subscriber odometry_sub_;
  //   ros::Subscriber pose_sub_;
  //   ros::Subscriber imu_sub_;
  //   ros::Subscriber cancel_sub_;

  //   // Publishers
  //   ros::Publisher collision_free_control_pub_;
  //   ros::Publisher cloud_before_pub_;
  //   ros::Publisher cloud_after_pub_;
  //   ros::Publisher cloud_obstacle_pub_;
  //   ros::Publisher debug_pub_;

  //   ros::Timer no_input_timer_;

  //   // Obstacles
  //   pcl::PointCloud<pcl::PointXYZ> obstacle_cloud_;

  //   // Polar Histogram
  //   double max_data_age_;
  //   int polar_size_;

  //   // Current pose
  //   geometry_msgs::PoseStamped current_pose_;

  //   // Pose when last setpoint recieved
  //   geometry_msgs::PoseStamped last_setpoint_;

  //   // Current imu;
  //   sensor_msgs::Imu imu_;

  //   // Current velocity
  //   double current_x_vel_;
  //   double current_y_vel_;

  //   // Obstacle-Restriction Method
  //   double radius_;
  //   double security_distance_;
  //   double epsilon_;

  //   // Robot
  //   std::string robot_frame_;
  //   double robot_height_;

  //   // Maximum velocities
  //   double max_xy_vel_;
  //   double max_z_vel_;
  //   double max_yaw_rate_;

  //   // Minimum velocities
  //   double min_xy_vel_;
  //   double min_z_vel_;
  //   double min_yaw_rate_;

  //   // Tele-op
  //   double min_change_in_direction_;
  //   double max_change_in_direction_;
  //   double min_opposite_direction_;
  //   double max_opposite_direction_;

  //   // Ego-Dynamic Space
  //   double ab_;
  //   double T_;

  //   // No input
  //   double min_distance_hold_;
  //   double closest_distance_;

  //   // Emperically measured constant for speed reduction
  //   double h_m_;

  // public:
  //   CollisionAvoidance(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  // private:
  //   void transformPointcloud(const std::string& target_frame, const std::string& source_frame,
  //                            const sensor_msgs::PointCloud2::ConstPtr& cloud_in, sensor_msgs::PointCloud2*
  //                            cloud_out);

  //   void rosToPcl(const sensor_msgs::PointCloud2& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

  //   void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

  //   // std::vector<Point> getPolarHistogram(const pcl::PointCloud<pcl::PointXYZ>& obstacle_cloud);

  //   std::vector<Eigen::Vector2d> getPolarHistogram(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  //   void postprocessPolarHistogram(std::vector<Eigen::Vector2d>* polar_hist);

  //   void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& setpoint);

  //   void avoidCollision(geometry_msgs::PoseStamped setpoint);

  //   void timerCallback(const ros::TimerEvent& event);

  //   std::vector<Point> getEgeDynamicSpace(const std::vector<Point>& obstacles_in);

  //   // void adjustVelocity(const std::vector<Point>& obstacles, geometry_msgs::TwistStamped* control,
  //   //                     const double magnitude);

  //   Eigen::Vector2d adjustVelocity(const Eigen::Vector2d& control, const std::vector<Eigen::Vector2d>& obstacles);

  //   void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom);

  //   void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);

  //   void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  //   void cancelCallback(const std_msgs::Header::ConstPtr& msg);

  //   void configCallback(collision_avoidance::CollisionAvoidanceConfig& config, uint32_t level);
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_H