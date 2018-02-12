#include <pluginlib/class_list_macros.h>

#include <collision_avoidance/collision_avoidance_nodelet.h>

#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/TransformStamped.h>

#include <pcl/PCLPointCloud2.h>

PLUGINLIB_EXPORT_CLASS(collision_avoidance::CANodelet, nodelet::Nodelet)

namespace collision_avoidance
{

    void CANodelet::onInit()
    {
        ros::NodeHandle & nh = getNodeHandle();
        ros::NodeHandle & nh_priv = getPrivateNodeHandle();

        init_param(nh_priv);

        //sensor_readings_sub_ = nh.subscribe("/sensor_readings", 1, &CANodelet::sensorReadingsCallback, this);
        collision_avoidance_setpoint_sub_ = nh.subscribe("/controller/setpoint", 1, &CANodelet::collisionAvoidanceSetpointCallback, this);
        collision_avoidance_joy_sub_ = nh.subscribe("/controller/joy", 1, &CANodelet::collisionAvoidanceJoyCallback, this);
        odometry_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 1, &CANodelet::odometryCallback, this);

        collision_free_control_pub_ = nh.advertise<controller_msgs::Controller>("collision_free_control", 1);

        orm_ = new ORM(radius_, security_distance_, epsilon_, min_distance_hold_, min_change_in_direction_, max_change_in_direction_, min_opposite_direction_, max_opposite_direction_);
    }

    void CANodelet::init_param(ros::NodeHandle & nh)
    {
        nh.param<double>("radius", radius_, 0);
        nh.param<double>("security_distance", security_distance_, 0);
        nh.param<double>("epsilon", epsilon_, 0.1);

        nh.param<double>("min_change_in_direction", min_change_in_direction_, -180);
        nh.param<double>("max_change_in_direction", max_change_in_direction_, 180);

        nh.param<double>("min_opposite_direction", min_opposite_direction_, -180);
        nh.param<double>("max_opposite_direction", max_opposite_direction_, 180);

        nh.param<double>("ab", ab_, 3.0);
        nh.param<double>("T", T_, 0.3);

        nh.param<double>("min_distance_hold", min_distance_hold_, 0.3);

        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    }

    void CANodelet::transformPointcloud(const std::string & target_frame, const std::string & source_frame, const sensor_msgs::PointCloud2::ConstPtr & in_cloud, sensor_msgs::PointCloud2 *out_cloud)
    {
      geometry_msgs::TransformStamped transform;
      try
      {
        transform = tf_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0));
      }
      catch (tf2::TransformException & ex)
      {
        NODELET_WARN("%s", ex.what());
        return;
      }

      tf2::doTransform(*in_cloud, *out_cloud, transform);

      out_cloud->header.frame_id = target_frame;
      out_cloud->header.stamp = in_cloud->header.stamp;
    }

    void CANodelet::rosToPcl(const sensor_msgs::PointCloud2 & in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
    {
      pcl::PCLPointCloud2 temp_cloud;
      pcl_conversions::toPCL(in_cloud, temp_cloud);
      pcl::fromPCLPointCloud2(temp_cloud, *out_cloud);
    }

    // Returns cloud.size() if this is the first time we get data from this sensor...
    size_t CANodelet::getStartingIndex(const pcl::PointCloud<PointXYZTF> & cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr new_data)
    {
      size_t i = 0;
      while (i < cloud.size())
      {
        if (cloud[i].how_many == new_data->width)
        {
          for (size_t j = 0; j < cloud[i].how_many; ++j)
          {
            if (!pcl_isfinite(cloud[i + j].x) || !pcl_isfinite(cloud[i + j].y) || !pcl_isfinite(cloud[i + j].z))
            {
              // Not a valid point
              continue;
            }
            if (!pcl_isfinite((*new_data)[i + j].x) || !pcl_isfinite((*new_data)[i + j].y) || !pcl_isfinite((*new_data)[i + j].z))
            {
              // Not a valid point
              continue;
            }
            // Both are valid, check if similar angle
            if (true)
            {
              return i;
            }
            else
            {
              break;
            }
          }
        }
        i += cloud[i].how_many;
      }

      return i;
    }

    void CANodelet::updateObstacleInformation(pcl::PointCloud<PointXYZTF> * obstacles, const pcl::PointCloud<pcl::PointXYZ>::Ptr new_data, size_t starting_index)
    {
      if (starting_index >= obstacles->size())
      {
        // This is (hopefully) the first time we got data from this sensor
        obstacles->resize(obstacles->size() + new_data->width);
      }

      // Insert the new data starting from starting_index to starting_index + new_data->width
      for (size_t i = 0; i < new_data->width; ++i)
      {
        PointXYZTF point;
        point.stamp = pcl_conversions::fromPCL(new_data->header.stamp);
        point.how_many = new_data->width;
        point.x = std::numeric_limits<float>::quiet_NaN();
        point.y = std::numeric_limits<float>::quiet_NaN();
        point.z = std::numeric_limits<float>::quiet_NaN();

        // Take the closest point in column that the robot can collied into
        for (size_t j = 0; j < new_data->height; ++j)
        {
          if (pcl_isfinite(new_data->at(i, j).z) && std::fabs(new_data->at(i, j).z) <= radius_)
          {
            // This point is at a height such that the robot can collied with it
            point.x = new_data->at(i, j).x;
            point.y = new_data->at(i, j).y;
            point.z = new_data->at(i, j).z;
          }
        }

        obstacles->points[starting_index + i] = point;
      }
    }

    void CANodelet::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
      // Transform msg to correct frame
      sensor_msgs::PointCloud2 transformed_cloud;
      transformPointcloud("base_link", msg->header.frame_id, msg, &transformed_cloud);

      // Convert it to PCL for easier access to elements
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      rosToPcl(transformed_cloud, pcl_cloud);

      // Check if we already have a message from this sensor (by checking how_many and
      // at which angle the data starts/stops)
      size_t starting_index = getStartingIndex(obstacle_cloud_, pcl_cloud);

      // Updated with new obstacle information
      updateObstacleInformation(&obstacle_cloud_, pcl_cloud, starting_index);
    }

    void CANodelet::collisionAvoidance(const controller_msgs::Controller::ConstPtr & msg, const double magnitude)
    {
        controller_msgs::Controller collision_free_control = *msg;

        std::vector<Point> obstacles; // = obstacles_;

        getEgeDynamicSpace(&obstacles);

        // First pass
        orm_->avoidCollision(&collision_free_control, magnitude, obstacles);

        // Second pass
        controller_msgs::Controller second_pass_collision_free_control = collision_free_control;

        orm_->avoidCollision(&second_pass_collision_free_control, magnitude, obstacles);

        // If there is a big difference between first and second pass then it is not safe to move!
        Point first_pass(collision_free_control.twist_stamped.twist.linear.x, collision_free_control.twist_stamped.twist.linear.y);
        Point second_pass(second_pass_collision_free_control.twist_stamped.twist.linear.x, second_pass_collision_free_control.twist_stamped.twist.linear.y);
        double dir_diff = std::fmod(std::fabs(Point::getDirectionDegrees(first_pass) - Point::getDirectionDegrees(second_pass)), 180.0);
        if (dir_diff > 140)
        {
          collision_free_control.twist_stamped.twist.linear.x = 0.0;
          collision_free_control.twist_stamped.twist.linear.y = 0.0;
          orm_->avoidCollision(&collision_free_control, magnitude, obstacles);
        }

        // Adjust the velocity
        adjustVelocity(&collision_free_control, magnitude);

        // Publish the collision free control
        collision_free_control_pub_.publish(collision_free_control);
    }

    void CANodelet::collisionAvoidanceSetpointCallback(const controller_msgs::Controller::ConstPtr & msg)
    {
        collisionAvoidance(msg, 1.0);
    }

    void CANodelet::collisionAvoidanceJoyCallback(const controller_msgs::Controller::Ptr & msg)
    {
        Point goal(msg->twist_stamped.twist.linear.x, msg->twist_stamped.twist.linear.y);

        // Multiple by a number to make a setpoint... x and y are between [0,1]
        msg->twist_stamped.twist.linear.x *= 10;
        msg->twist_stamped.twist.linear.y *= 10;

        collisionAvoidance(msg, Point::getDistance(goal));
    }

    void CANodelet::getEgeDynamicSpace(std::vector<Point> * obstacles)
    {
//        for (size_t i = 0; i < obstacles_.size(); ++i)
//        {
//            if (obstacles_[i].x_ == 0 && obstacles_[i].y_ == 0)
//            {
//                // No reading here
//                obstacles->push_back(obstacles_[i]);
//                continue;
//            }

//            Point p;
//            p.x_ = obstacles_[i].x_ - current_x_vel_;
//            p.y_ = obstacles_[i].y_ - current_y_vel_;

//            // TODO: Check that it is above 0

//            obstacles->push_back(p);
//        }
    }

    void CANodelet::adjustVelocity(controller_msgs::Controller * control, const double magnitude)
    {
        double closest_obstacle_distance = 1000;
        for (size_t i = 0; i < obstacles_.size(); ++i)
        {
//            double distance = Point::getDistance(obstacles_[i]);

//            if (distance != 0 && distance < closest_obstacle_distance)
//            {
//                closest_obstacle_distance = distance;
//            }
        }

        Point goal(control->twist_stamped.twist.linear.x, control->twist_stamped.twist.linear.y);
        double direction = Point::getDirectionDegrees(goal);

        double updated_magnitude = std::min(closest_obstacle_distance / (radius_ + security_distance_), magnitude);

        Point updated_goal = Point::getPointFromVectorDegrees(direction, updated_magnitude);

        control->twist_stamped.twist.linear.x = updated_goal.x_;
        control->twist_stamped.twist.linear.y = updated_goal.y_;
    }

    void CANodelet::odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        current_x_vel_ = msg->twist.twist.linear.y; // This should be y?
        current_y_vel_ = msg->twist.twist.linear.x; // This should be x?
    }
}
