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

//#include <sensor_readings/SensorReadings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <controller_msgs/Controller.h>

namespace collision_avoidance
{
    class CANodelet : public nodelet::Nodelet
    {

    private:
        // Obstacles
        std::vector<Point> obstacles_;
        std::vector<double> new_obstacles_;
        std::vector<ros::Time> obstacles_acquired_;

        std::vector<pcl::PointCloud<pcl::PointXYZ> > obstacle_cloud_;

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
        std::vector<ros::Subscriber> obstacle_sub_;
        ros::Subscriber collision_avoidance_joy_sub_;
        ros::Subscriber collision_avoidance_setpoint_sub_;
        ros::Subscriber odometry_sub_;

        // Publishers
        ros::Publisher collision_free_control_pub_;
        ros::Publisher rumble_pub_;

        // Transform
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener * tf_listener_;

    private:
        virtual void onInit();

        void init_param(ros::NodeHandle & nh);

        void transformPointcloud(const std::string & target_frame, const std::string & source_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud, sensor_msgs::PointCloud2 * out_cloud);

        void rosToPcl(const sensor_msgs::PointCloud2 & in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);

        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & cloud, int index);

        void collisionAvoidance(const controller_msgs::Controller::ConstPtr & msg, const double magnitude);

        void collisionAvoidanceSetpointCallback(const controller_msgs::Controller::ConstPtr & msg);

        void collisionAvoidanceJoyCallback(const controller_msgs::Controller::Ptr & msg);

        void getEgeDynamicSpace(std::vector<Point> * obstacles);

        void adjustVelocity(controller_msgs::Controller * control, const double magnitude);

        void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
    };
    
}
