#!/usr/bin/env python

# Import ROS packages
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
import tf2_ros

# Import other required packages
import numpy as np
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

import time


class PathPlanner:

    def __init__(self, path_topic, world_frame_id, robot_frame_id):

        # Variables to store current start and goal poses as well as world map
        self.goal_pose = None
        self.map = None

        # Publisher to send new routes once they are computed
        self.route_pub = rospy.Publisher(path_topic, Path, queue_size=0)

        # Create transform listener for retrieving pose information
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Frames to use to get position update
        self.world_frame_id = world_frame_id
        self.robot_frame_id = robot_frame_id

    # Update the goal position for the route (triggers route generation)
    def update_goal_pose(self, pose_msg):
        self.goal_pose = pose_msg
        self.generate_route()

    # Update the map which routes are computed relative to (triggers route generation)
    def update_map(self, new_map_msg):

        # Get new map, make sure dimensions are correct
        map_length = new_map_msg.info.height
        map_width = new_map_msg.info.width

        # Convert to numpy array and store
        self.map = np.reshape(np.array(new_map_msg.data), (map_length, map_width))

    # Compute a new route from start_pose to goal_pose and publish
    # Uses A* search algorithm to compute the route
    def generate_route(self):

        if self.goal_pose is not None and self.map is not None:

            start_time = time.time()

            # Get latest pose from transform tree
            cur_pose = self.tf_buffer.lookup_transform(self.world_frame_id, self.robot_frame_id, rospy.Time())

            # Extract information from pose message
            position = np.array([cur_pose.transform.translation.x, cur_pose.transform.translation.y])

            # Create grid object from current map
            grid = Grid(matrix=self.map)

            # Define nodes for start position and target position
            start = grid.node(int(position[0] * 100), int(position[1] * 100))
            end = grid.node(int(self.goal_pose.position.x * 100), int(self.goal_pose.position.y * 100))

            # Create an A-star path finder object
            finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

            # Compute path
            path, _ = finder.find_path(start, end, grid)

            # Convert to a ROS Path object
            new_path = Path()
            new_path.header.frame_id = self.world_frame_id

            for point in path:

                # Create new pose
                new_pose = PoseStamped()
                new_pose.header.frame_id = self.world_frame_id
                new_pose.pose.position.x = (point[0] / 100.0)
                new_pose.pose.position.y = (point[1] / 100.0)

                # Add to list of poses
                new_path.poses.append(new_pose)

            # Publish the path message
            self.route_pub.publish(new_path)

            rospy.loginfo(str(time.time() - start_time))


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("path_planning")

    # Read parameters from server
    goal_pose_topic = rospy.get_param("goal_pose_topic", "goal_pose")
    path_topic = rospy.get_param("path_topic", "cur_path")
    map_topic = rospy.get_param("map_topic", "cur_map")
    world_frame_id = rospy.get_param("pp_world_frame_id", "world")
    robot_frame_id = rospy.get_param("pp_robot_frame_id", "robot_center")

    # Create path planner object
    planner = PathPlanner(path_topic, world_frame_id, robot_frame_id)

    # Create subscribers to update start / goal poses & map
    rospy.Subscriber(goal_pose_topic, Pose, planner.update_goal_pose)
    rospy.Subscriber(map_topic, OccupancyGrid, planner.update_map)

    # Update the route once per second
    loop_rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        # Generate new route and sleep
        planner.generate_route()
        loop_rate.sleep()


