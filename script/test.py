#! /usr/bin/env python

import rospy
from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import collision_avoidance.msg

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


def collision_avoidance_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient(
        'move_to', collision_avoidance.msg.PathControlAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = rospy.Time.now()

    pose = PoseStamped()
    pose.header = path.header
    pose.pose.position.x = 1
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    pose.pose.orientation = quaternion_from_euler(0.0, 0.0, 0.0)

    path.poses.append(pose)

    # Creates a goal to send to the action server.
    goal = collision_avoidance.msg.PathControlGoal(path=path)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('collision_avoidance_test')
        collision_avoidance_client()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
