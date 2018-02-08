#pragma once

#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <collision_avoidance/point.h>
#include <collision_avoidance/obstacle_restriction_method.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_readings/SensorReadings.h>
#include <controller_msgs/Controller.h>

namespace collision_avoidance
{
    
    class CANodelet : public nodelet::Nodelet
    {

    private:
        // Obstacles
        std::vector<Point> obstacles_;

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

        void sensorReadingsCallback(const sensor_readings::SensorReadings::ConstPtr & msg);

        void collisionAvoidance(const controller_msgs::Controller::ConstPtr & msg, const double magnitude);

        void collisionAvoidanceSetpointCallback(const controller_msgs::Controller::ConstPtr & msg);

        void collisionAvoidanceJoyCallback(const controller_msgs::Controller::Ptr & msg);

        void getEgeDynamicSpace(std::vector<Point> * obstacles);

        void adjustVelocity(controller_msgs::Controller * control, const double magnitude);

        void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
    };
    
}
