#!/usr/bin/env python

# Import ROS packages
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid

# Import other required packages
import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


class PathPlanner:

    def __init__(self, path_topic, map_length, map_width):

        # Variables to store current start and goal poses as well as world map
        self.start_pose = Pose()
        self.goal_pose = Pose()
        self.map_length = map_length
        self.map_width = map_width
        self.map = np.zeros(shape=(map_length, map_width))

        # Publisher to send new routes once they are computed
        self.route_pub = rospy.Publisher(path_topic, Path, queue_size=0)

    # Update the starting position for new routes
    def update_start_pose(self, pose_msg):
        self.start_pose = pose_msg

    # Update the goal position for the route (triggers route generation)
    def update_goal_pose(self, pose_msg):
        self.goal_pose = pose_msg
        self.generate_route()

    # Update the map which routes are computed relative to (triggers route generation)
    def update_map(self, new_map_msg):

        # Get new map, make sure dimensions are correct
        self.map_length = new_map_msg.info.height
        self.map_width = new_map_msg.info.width

        # Convert to numpy array and store
        self.map = np.reshape(np.array(new_map_msg.data), (self.map_length, self.map_width))

    # Compute a new route from start_pose to goal_pose and publish
    # Uses A* search algorithm to compute the route
    def generate_route(self):

        # Create grid object from current map
        grid = Grid(matrix=self.map)

        # Define nodes for start position and target position
        start = grid.node(self.start_pose.position.x, self.start_pose.position.y)
        end = grid.node(self.goal_pose.position.x, self.goal_pose.position.y)

        # Create an A-star path finder object
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

        # Compute path
        path, _ = finder.find_path(start, end, grid)

        # Convert to a ROS Path object
        new_path = Path()

        for point in path:

            # Create new pose
            new_pose = PoseStamped()
            new_pose.header.frame_id = "world"
            new_pose.pose.position.x = point[0]
            new_pose.pose.position.y = point[1]

            # Add to list of poses
            new_path.poses.append(new_pose)

        # Publish the path message
        self.route_pub.publish(path)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("path_planning")

    # Read parameters from server
    start_pose_topic = rospy.get_param("start_pose_topic", "cur_pose")
    goal_pose_topic = rospy.get_param("goal_pose_topic", "goal_pose")
    path_topic = rospy.get_param("path_topic", "cur_path")
    map_topic = rospy.get_param("map_topic", "cur_map")

    # Create path planner object
    planner = PathPlanner(path_topic, map_length=500, map_width=500)

    # Create subscribers to update start / goal poses & map
    rospy.Subscriber(start_pose_topic, Pose, planner.update_start_pose)
    rospy.Subscriber(goal_pose_topic, Pose, planner.update_goal_pose)
    rospy.Subscriber(map_topic, OccupancyGrid, planner.update_map)

    # Update the route once per second
    loop_rate = rospy.Rate(2)
    while not rospy.is_shutdown():

        # Generate new route and sleep
        planner.generate_route()
        loop_rate.sleep()


