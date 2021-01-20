#ifndef COLLISION_AVOIDANCE_H
#define COLLISION_AVOIDANCE_H

// UFO
#include <ufo/map/occupancy_map.h>
#include <ufo/map/occupancy_map_color.h>

// UFO ROS
#include <ufomap_msgs/UFOMapMetaData.h>
#include <ufomap_msgs/UFOMapStamped.h>
#include <ufomap_msgs/conversions.h>

// ROS
#include <actionlib/server/simple_action_server.h>
#include <collision_avoidance/CollisionAvoidanceConfig.h>
#include <collision_avoidance/PathControlAction.h>
#include <collision_avoidance/obstacle_restriction_method.h>
#include <collision_avoidance/polar_histogram.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

// STD
#include <shared_mutex>
#include <variant>

namespace collision_avoidance
{
class CollisionAvoidance
{
 private:
	// Subscribers
	ros::Subscriber map_sub_;
	std::vector<ros::Subscriber> cloud_sub_;
	ros::Subscriber odometry_sub_;

	// Publishers
	ros::Publisher control_pub_;
	ros::Publisher path_pub_;
	ros::Publisher obstacle_pub_;
	ros::Publisher current_setpoint_;

	// Action servers
	actionlib::SimpleActionServer<collision_avoidance::PathControlAction> as_;

	// TF2
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	// Dynamic reconfigure
	dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig> cs_;
	dynamic_reconfigure::Server<collision_avoidance::CollisionAvoidanceConfig>::CallbackType
	    f_;

	//
	// UFOMap parameters
	//

	// UFOMap
	std::variant<std::monostate, ufo::map::OccupancyMap, ufo::map::OccupancyMapColor> map_;
	// Automatic pruning
	bool automatic_pruning_;
	// Occupied threshold
	double occupied_thres_;
	// Free threshold
	double free_thres_;
	// Treat unknown as occupied
	bool unknown_as_occupied_;
	// UFOMap frame
	std::string map_frame_id_;
	// Depth to use for UFOMap calculations
	ufo::map::DepthType map_depth_;

	// Stored data
	std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> clouds_;
	nav_msgs::Odometry::ConstPtr odometry_;

	// Mutex
	mutable std::shared_mutex map_mutex_;

	// Current target
	geometry_msgs::PoseStamped target_;

	// ORM
	ORM orm_;

	// Frames
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
	double h_m_;

	//
	double max_direction_change_;
	int max_times_backwards_;

	int num_histogram_;

	double radius_;
	double height_;
	double min_distance_hold_;

	double leaf_size_;

	double look_ahead_distance_;
	bool look_forward_;
	bool move_while_yawing_;
	bool yaw_each_setpoint_;

 public:
	CollisionAvoidance(ros::NodeHandle &nh, ros::NodeHandle &nh_priv);

 private:
	void goalCallback(const collision_avoidance::PathControlGoal::ConstPtr &goal);

	geometry_msgs::PoseStamped getNextSetpoint(nav_msgs::Path *path,
	                                           bool go_to_every_target = false) const;

	double getDistanceClosestObstacle(double obstacle_window, double height_diff) const;

	geometry_msgs::Pose interpolate(const geometry_msgs::Pose &start,
	                                const geometry_msgs::Pose &end, double t) const;

	geometry_msgs::Point lerp(const geometry_msgs::Point &start,
	                          const geometry_msgs::Point &end, double t) const;

	geometry_msgs::Quaternion slerp(const geometry_msgs::Quaternion &start,
	                                const geometry_msgs::Quaternion &end, double t) const;

	bool avoidCollision(geometry_msgs::PoseStamped setpoint, bool do_avoidance = true);

	void noInput(geometry_msgs::PoseStamped setpoint) const;

	void adjustVelocity(geometry_msgs::TwistStamped *control,
	                    const PolarHistogram &obstacles) const;

	PolarHistogram getObstacles(double obstacle_window, double height_diff = 0.0) const;

	std::pair<double, double> getDistanceToTarget(const geometry_msgs::PoseStamped &target);

	void timerCallback(const ros::TimerEvent &event);

	void publishObstacles(const PolarHistogram &obstacles) const;

	void cloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, int index)
	{
		clouds_[index] = cloud;
	}

	void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg)
	{
		if (!checkMap(msg->map.info.id, msg->map.info.resolution,
		              msg->map.info.depth_levels)) {
			if (!createMap(msg->map.info)) {
				fprintf(stderr, "Could not create map!\n");
				// TODO: ERROR
				return;
			}
		}

		if (!std::visit(
		        [this, &msg](auto &map) -> bool {
			        if constexpr (!std::is_same_v<std::decay_t<decltype(map)>,
			                                      std::monostate>) {
				        return ufomap_msgs::msgToUfo(msg->map, map);
			        }
			        return false;
		        },
		        map_)) {
			fprintf(stderr, "Could not convert msg to map!\n");
			// TODO: ERROR
			return;
		}
	}

	void odometryCallback(const nav_msgs::Odometry::ConstPtr &odometry)
	{
		odometry_ = odometry;
	}

	void configCallback(const collision_avoidance::CollisionAvoidanceConfig &config,
	                    uint32_t level);

	bool createMap(ufomap_msgs::UFOMapMetaData const &info)
	{
		// FIXME: Remove hardcoded
		if ("occupancy_map" == info.id) {
			std::unique_lock write_lock(map_mutex_);
			map_.emplace<ufo::map::OccupancyMap>(info.resolution, info.depth_levels,
			                                     automatic_pruning_, occupied_thres_,
			                                     free_thres_);
			return true;
		} else if ("occupancy_map_color" == info.id) {
			std::unique_lock write_lock(map_mutex_);
			map_.emplace<ufo::map::OccupancyMapColor>(info.resolution, info.depth_levels,
			                                          automatic_pruning_, occupied_thres_,
			                                          free_thres_);
			return true;
		}

		return false;
	}

	bool checkMap(std::string const &type, double resolution,
	              ufo::map::DepthType depth_levels) const
	{
		return std::visit(
		    [this, &type, resolution, depth_levels](auto &map) -> bool {
			    if constexpr (!std::is_same_v<std::decay_t<decltype(map)>, std::monostate>) {
				    return map.getTreeType() == type && map.getResolution() == resolution &&
				           map.getTreeDepthLevels() == depth_levels;
			    }
			    return false;
		    },
		    map_);
	}
};
}  // namespace collision_avoidance

#endif  // COLLISION_AVOIDANCE_H