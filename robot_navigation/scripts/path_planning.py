#!/usr/bin/env python

# Import ROS packages
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header
import tf2_ros

# A-star path finding library
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# Other important packages
import time
import numpy as np


# Check if input point is within the field boundaries
def is_in_field(driveability_map, pt):
    return True if ((pt >= 0).all() and pt[0] < driveability_map.shape[1] and pt[1] < driveability_map.shape[0]) \
        else False


# Checks if the path between two points is clear
# Drivability map is in cm
# Start / end pos are in meters
def is_clear_path(driveability_map, start_pos, end_pos):

    # Convert start and end positions to cm
    start_pos = [int(i * 100) for i in start_pos]
    end_pos = [int(i * 100) for i in end_pos]

    # Set step size to 1 so we don' miss any obstacles
    step_size = 1

    # calculating length of the straight line and the number of points
    dy = end_pos[1] - start_pos[1]
    dx = end_pos[0] - start_pos[0]
    path_length = np.sqrt(dx ** 2 + dy ** 2)
    matrix_len = int(path_length / step_size)

    # creating matrices for calculations
    points = [start_pos for i in range(matrix_len)]
    steps = [[step_size * dx / path_length, step_size * dy / path_length] for i in range(matrix_len)]
    nums = [[i + 1, i + 1] for i in range(matrix_len)]

    # Calculating the path
    path = np.add(points, np.multiply(steps, nums))

    # Prepend starting point
    path = np.insert(path, 0, start_pos, axis=0)

    # Check for obstacles
    for pt in path:

        # Check boundaries
        if not is_in_field(driveability_map, pt):
            return False

        if driveability_map[int(pt[1]), int(pt[0])] == 1:
            return False  # obstacle detected, can't cross here

    return True  # Safe to drive along this line


# Generates a ROS Path message for a path between two points - assumes that
# there are no obstacles on the path. Points on the path separated by step_size
def compute_easy_path(start_pos, end_pos, step_size):

    # calculating length of the straight line and the number of points
    dy = end_pos[1] - start_pos[1]
    dx = end_pos[0] - start_pos[0]
    path_length = np.sqrt(dx ** 2 + dy ** 2)
    matrix_len = int(path_length / step_size)

    # creating matrices for calculations
    points = [start_pos for i in range(matrix_len)]
    steps = [[step_size * dx / path_length, step_size * dy / path_length] for i in range(matrix_len)]
    nums = [[i + 1, i + 1] for i in range(matrix_len)]

    # Calculating the path
    path = np.add(points, np.multiply(steps, nums))

    # returning ROS Path object
    path_msg = Path()
    path_msg.header.frame_id = "world"

    for point in path:

        waypoint = PoseStamped()
        waypoint.header.frame_id = "world"
        waypoint.pose.position.x = point[0]
        waypoint.pose.position.y = point[1]

        path_msg.poses.append(waypoint)

    return path_msg


# Computes a 'difficult' path between two points, provided with an obstacle map - does NOT assume
# that there are no obstacles between the points.
# K = resolution reduction factor to apply - reduces computation time significantly
def compute_hard_path(map, start_pt, end_pt, K, step_size):

    # First, trim the map down such that it can be divided evenly into K by K square sections.
    # Try to keep the trimming as symmetric as possible: If we trim the bottom side, trim the top side next, etc.
    H, W = map.shape
    K = K

    H_excess = H % K
    W_excess = W % K

    start_x = H_excess / 2
    end_x = H - (H_excess / 2)

    start_y = W_excess / 2
    end_y = W - (W_excess / 2)

    # In the event that we only need to trim one edge to make that dimension divisible by K, we have over-adjusted
    # in the above code. Rectify that here - is there a simple way to not make that mistake prior?
    if (H_excess % 2 == 1):
        end_x -= 1

    if (W_excess % 2 == 1):
        end_y -= 1

    map = map[start_x:end_x, start_y:end_y]  # Adjusted map that can now be divided into KxK sections

    # Divide the adjusted map into KxK sections, taking the max value of each section to be the value of that
    # section.
    # We can also take a running total of the number of 1's in each section, to determine which
    # sections are least likely to be impassable.
    HK = H // K
    WK = W // K

    weighted_map = (map[:HK * K, :WK * K].reshape(HK, K, WK, K).sum(axis=(1, 3)))
    weighted_map[weighted_map > 0] *= -1
    weighted_map[weighted_map == 0] = 1

    # Create Grid object from map for A-star pathfinder to use
    grid = Grid(matrix=weighted_map)

    # Define nodes for start and end positions - scale to map units (m -> cm)
    start = grid.node(int(start_pt[0] / K * 100), int(start_pt[1] / K * 100))
    end = grid.node(int(end_pt[0] / K * 100), int(end_pt[1] / K * 100))

    # Create an A-star finder object
    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)

    # Compute the path
    path, _ = finder.find_path(start, end, grid)

    # Check if we found a path
    path_found = (len(path) != 0)
    threshold = 0

    # Keep relaxing constraints until a path is found
    while not path_found:

        # Reduce threshold
        threshold -= 1

        # Adjust map to match threshold
        weighted_map[weighted_map == threshold] = 1
        grid = Grid(matrix=weighted_map)

        # Compute path
        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, _ = finder.find_path(start, end, grid)

        # Check if we found a path
        path_found = (len(path) != 0)

    # Convert the path to a numpy array and scale back up to original dimensions
    adj_path = np.array(path)
    adj_path = (K * adj_path + (K/2)) / 100.0

    # Initialize full path
    full_path = Path()
    full_path.header.frame_id = "world"

    # Build ROS path from the computed path
    for i in range(len(adj_path) - 1):

        # Compute 'easy' path for current segment
        segment_path = compute_easy_path(adj_path[i], adj_path[i+1], step_size)

        # Add segment to the full path
        full_path.poses = full_path.poses + segment_path.poses

    return full_path


# Class for managing the path planning logic
class PathPlanner:

    def __init__(self, path_topic, world_frame_id, robot_frame_id):

        # Variables to store current start and goal poses as well as world map
        self.goal_point = None
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
    def update_goal_point(self, point_msg):

        self.goal_point = point_msg
        self.generate_route()

    # Update the map which routes are computed relative to (triggers route generation)
    def update_map(self, new_map_msg):

        # Get new map, make sure dimensions are correct
        map_length = new_map_msg.info.height
        map_width = new_map_msg.info.width

        # Convert to numpy array and store
        self.map = np.reshape(np.array(new_map_msg.data), (map_length, map_width))

    # Compute a new route from start_pose to goal_point and publish
    # Uses A* search algorithm to compute the route
    def generate_route(self):

        if self.goal_point is not None and self.map is not None:

            # Get latest pose from transform tree
            cur_pose = self.tf_buffer.lookup_transform(self.world_frame_id, self.robot_frame_id, rospy.Time())

            # Extract information from pose message
            position = [cur_pose.transform.translation.x, cur_pose.transform.translation.y]
            target = [self.goal_point.point.x, self.goal_point.point.y]

            # Check if the path is easy
            if is_clear_path(self.map, position, target):
                new_path = compute_easy_path(position, target, 0.04)

            # Else, compute hard path
            else:
                new_path = compute_hard_path(self.map, position, target, 10, 0.04)

            # Publish the path message
            self.route_pub.publish(new_path)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("path_planning")

    # Read parameters from server
    goal_point_topic = rospy.get_param("goal_point_topic", "goal_point")
    path_topic = rospy.get_param("path_topic", "cur_path")
    map_topic = rospy.get_param("map_topic", "cur_map")
    world_frame_id = rospy.get_param("pp_world_frame_id", "world")
    robot_frame_id = rospy.get_param("pp_robot_frame_id", "robot_center")

    # Create path planner object
    planner = PathPlanner(path_topic, world_frame_id, robot_frame_id)

    # Create subscribers to update start / goal poses & map
    rospy.Subscriber(goal_point_topic, PointStamped, planner.update_goal_point)
    rospy.Subscriber(map_topic, OccupancyGrid, planner.update_map)

    # Update the route once every two seconds (or when a new goal_pos received)
    loop_rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():

        # Generate new route and sleep
        planner.generate_route()
        loop_rate.sleep()


