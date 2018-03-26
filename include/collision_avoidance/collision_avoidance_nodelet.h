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

#include <controller_msgs/Controller.h>

namespace collision_avoidance
{
    class CANodelet : public nodelet::Nodelet
    {

    private:
        // Obstacles
        std::vector<pcl::PointCloud<pcl::PointXYZ> > obstacle_cloud_;

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
        ORM * orm_;
        double radius_;
        double security_distance_;
        double epsilon_;

        //
        double height_;

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
        ros::Subscriber collision_avoidance_joy_sub_;
        ros::Subscriber collision_avoidance_setpoint_sub_;
        ros::Subscriber odometry_sub_;
        ros::Subscriber imu_sub_;

        // Publishers
        ros::Publisher collision_free_control_pub_;
        ros::Publisher cloud_before_pub_;
        ros::Publisher cloud_after_pub_;
        ros::Publisher cloud_obstacle_pub_;

        // Transform
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener * tf_listener_;

    private:
        virtual void onInit();

        void init_param(ros::NodeHandle & nh);

        void transformPointcloud(const std::string & target_frame, const std::string & source_frame, const sensor_msgs::PointCloud2::ConstPtr & cloud_in, sensor_msgs::PointCloud2 * cloud_out);

        void rosToPcl(const sensor_msgs::PointCloud2 & cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & cloud, int index);

        std::vector<Point> getPolarHistogram(const std::vector<pcl::PointCloud<pcl::PointXYZ> > & obstacle_cloud);

        void collisionAvoidance(const controller_msgs::Controller::ConstPtr & msg, const double magnitude);

        void collisionAvoidanceSetpointCallback(const controller_msgs::Controller::ConstPtr & msg);

        void collisionAvoidanceJoyCallback(const controller_msgs::Controller::Ptr & msg);

        std::vector<Point> getEgeDynamicSpace(const std::vector<Point> & obstacles_in);

        void adjustVelocity(const std::vector<Point> & obstacles, controller_msgs::Controller * control, const double magnitude);

        void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);

        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
    };
    
}
