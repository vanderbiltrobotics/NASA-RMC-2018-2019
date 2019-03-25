#!/usr/bin/env python

# Import ROS packages
import rospy
import astar
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from nav_msgs.srv import GetMap

# Import other required packages
import numpy as np
from matplotlib import pyplot as plt

class PathPlanner:

    def __init__(self, path_topic, map_service):

        # Variables to store current start and goal poses as well as world map
        self.start_pose = Pose()
        self.goal_pose = Pose()
        self.map = OccupancyGrid()
        self.map_length = 0
        self.map_width = 0

        # Publisher to send new routes once they are computed
        self.route_pub = rospy.Publisher(path_topic, Path, queue_size=0)

        # Service client for requesting latest map
        self.get_map = rospy.ServiceProxy(map_service, GetMap)
        self.update_map()

    # Update the starting position for new routes
    def update_start_pose(self, pose_msg):
        self.start_pose = pose_msg

    # Update the goal position for the route (triggers route generation)
    def update_goal_pose(self, pose_msg):
        self.goal_pose = pose_msg
        self.generate_route()

    # Update the map which routes are computed relative to (triggers route generation)
    def update_map(self):

        # Get new map, make sure dimensions are correct
        new_map = self.get_map().map
        self.map_length = new_map.info.height
        self.map_width = new_map.info.width

        # Convert to numpy array and store
        self.map = np.reshape(np.array(new_map.data), (self.map_length, self.map_width))

    # Compute a new route from start_pose to goal_pose and publish
    # Uses A* search algorithm to compute the route
    def generate_route(self):
        path = astar.AStar()
        self.map = path.astar(self.start_pose,self.goal_pose)
        print path


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("path_planning")

    # Read parameters from server
    start_pose_topic = rospy.get_param("start_pose_topic", "cur_pose")
    goal_pose_topic = rospy.get_param("goal_pose_topic", "goal_pose")
    path_topic = rospy.get_param("path_topic", "cur_path")
    map_service = rospy.get_param("map_topic", "static_map")

    # Create path planner object
    planner = PathPlanner(path_topic, map_service)

    # Create subscribers to update start / goal poses & map
    rospy.Subscriber(start_pose_topic, Pose, planner.update_start_pose)
    rospy.Subscriber(goal_pose_topic, Pose, planner.update_goal_pose)

    # Spin until ROS shuts down or we stop node
    rospy.spin()