#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <collision_avoidance/point.h>
#include <collision_avoidance/obstacle_restriction_method.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

//#include <sensor_readings/SensorReadings.h>
#include <sensor_msgs/LaserScan.h>
#include <controller_msgs/Controller.h>

namespace collision_avoidance
{
  // http://pointclouds.org/documentation/tutorials/adding_custom_ptype.php#how-to-add-a-new-pointt-type
  struct PointXYZTF
  {
    PCL_ADD_POINT4D;
    ros::Time stamp;
    int how_many;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
    
    class CANodelet : public nodelet::Nodelet
    {

    private:
        // Obstacles
        std::vector<Point> obstacles_;
        std::vector<double> new_obstacles_;
        std::vector<ros::Time> obstacles_acquired_;

        pcl::PointCloud<PointXYZTF> obstacle_cloud_;

        // Current pose
        geometry_msgs::PoseStamped current_pose_;

        // Current velocity
        double current_x_vel_;
        double current_y_vel_;

        // Obstacle-Restriction Method
        ORM * orm_;
        double radius_;
        double security_distance_;
        double epsilon_;

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
        ros::Subscriber sensor_readings_sub_;
        ros::Subscriber collision_avoidance_joy_sub_;
        ros::Subscriber collision_avoidance_setpoint_sub_;
        ros::Subscriber odometry_sub_;

        // Publishers
        ros::Publisher collision_free_control_pub_;
        ros::Publisher rumble_pub_;

    private:
        virtual void onInit();

        void init_param(ros::NodeHandle & nh);

        void pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg);

//        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr & msg);

        //void sensorReadingsCallback(const sensor_readings::SensorReadings::ConstPtr & msg);

        void collisionAvoidance(const controller_msgs::Controller::ConstPtr & msg, const double magnitude);

        void collisionAvoidanceSetpointCallback(const controller_msgs::Controller::ConstPtr & msg);

        void collisionAvoidanceJoyCallback(const controller_msgs::Controller::Ptr & msg);

        void getEgeDynamicSpace(std::vector<Point> * obstacles);

        void adjustVelocity(controller_msgs::Controller * control, const double magnitude);

        void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
    };
    
}
