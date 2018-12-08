#!/usr/bin/env python

# ----------------------------- #
# DESCRIPTION OF THIS NODE HERE #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Pose, Point, Twist


class RoutePlanner:

    # Constructor
    def __init__(self):

        # State variables
        self.goal_pose = None
        self.robot_pose = None
        self.robot_twist = None
        self.map = None

        # Initialize publishers
        rospy.Publisher('drive_cmd', Pose, queue_size=1)

        # Initialize subscribers
        self.goal_pose_sub = rospy.Subscriber('goal_pose', Pose, self.update_goal_pose)
        self.robot_pose_sub = rospy.Subscriber('SLAM_pose_est', Pose, self.update_robot_pose)
        self.obs_sub = rospy.Subscriber('SLAM_obs', Point, self.update_map)
        self.twist_sub = rospy.Subscriber('SLAM_twist_est', Twist, self.update_robot_twist)


    # Callback function for incoming goal poses
    # Updates the planners current goal pose and immediately
    # triggers a new drive command update
    def update_goal_pose(self, msg):
        pass


    # WE SHOULD PROBABLLY BUNDLE ALL THE SLAM MESSAGES INTO A 
    # SINGLE SLAM UPDATE MESSAGE

    # Callback function for incoming pose estimates
    def update_robot_pose(self, msg):
        pass

    # Callback function for incoming twist estimates
    def update_robot_twist(self, msg):
        pass

    # Callback function for incoming obstacle data updates
    def update_map(self, msg):
        pass

# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('route_planning_execution')

    # Create a RoutePlanner object
    navigator = RoutePlanner()

    # Ready to go
    rospy.loginfo("Route Planner initialized...")

    # Loop continuously
    while not rospy.is_shutdown():
        pass