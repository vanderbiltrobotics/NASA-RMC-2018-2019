#!/usr/bin/env python

# ------------------------------------------------------------------------- #
#
# This node listens to updates about the robot's pose, goal pose, and
# velocities and produces image representations of the current state of the
# competition area based on that information. These images are intended to
# be displayed by the 'command center' interface to give operators a visual
# representation of what's happening during the competition.
#
# ------------------------------------------------------------------------- #

# Import ROS packages
import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, rotation_matrix
from cv_bridge import CvBridge

# Import other useful packages
from math import sin, cos
import numpy as np
import cv2


# ------------------------------------------------------------------------- #
#
# This function creates an image representing the competition area for the
# mining competition. All relevant dimensions can be passed as parameters
# but defaults are provided to reflect the current state of the rules. The
# scale parameter determines the pixels / meter for the image produced. It
# must be consistent across all drawing functions used or results will be
# incorrect
#
# ------------------------------------------------------------------------- #
def generate_base_img(fh=5.76, fw=3.69, dzl=1.92, ozl=3.84, bh=1.65, bw=0.48, bo=0.50, bc='left', scale=100):

    # Initialize full image
    img = np.zeros((int(fh * scale), int(fw * scale), 3), np.uint8) + 244

    # Add horizontal lines demarcating regions
    cv2.line(img, (0, int(dzl * scale)), (int(fw * scale), int(dzl * scale)), (175, 175, 175), 1)
    cv2.line(img, (0, int(ozl * scale)), (int(fw * scale), int(ozl * scale)), (175, 175, 175), 1)

    # Calculate corner coords for collection bin
    bin_x1 = 0
    bin_y1 = (fh - bo) * scale
    bin_y2 = bin_y1 - (bh * scale)

    # Adjust for field configuration
    if bc == 'right':
        bin_x1 += (fw - bw) * scale
    bin_x2 = bin_x1 + (bw * scale)

    # Draw collection bin
    cv2.rectangle(img, (int(bin_x1), int(bin_y1)), (int(bin_x2), int(bin_y2)), (255, 231, 198), cv2.FILLED)
    cv2.rectangle(img, (int(bin_x1), int(bin_y1)), (int(bin_x2), int(bin_y2)), (160, 122, 69), 2)

    # Return image
    return img


# ------------------------------------------------------------------------- #
#
# This adds a robot symbol to a provided 'base image' (which can be produced
# with 'generate_base_img'). The symbol includes a box showing the outer
# dimensions of the robot, as well as an arrow to show the current pose of
# the robot. Robot dimensions can be passed as parameters. The colors for
# the robot outline and pose arrow can also be adjusted. Scale determines the
# pixels / meter for the image.
#
# ------------------------------------------------------------------------- #
def draw_robot(base_img, pose, rw=0.6, rl=0.8, r_color=(0,0,0), p_color=(0,150,0), scale=100):

    # Initialize new image from base image
    img = np.copy(base_img)

    # Get quaternion from pose message
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Convert quaternion rotation to rpy, extract yaw
    theta = euler_from_quaternion(quat)[2]

    # Draw arrow representing robot pose
    arrow_x1 = pose.position.x
    arrow_y1 = pose.position.y
    arrow_x2 = arrow_x1 + (cos(theta) * .35)
    arrow_y2 = arrow_y1 - (sin(theta) * .35)
    cv2.arrowedLine(img, (int(arrow_x1 * scale), int(arrow_y1 * scale)),
                         (int(arrow_x2 * scale), int(arrow_y2 * scale)), p_color, 2, tipLength=0.15)

    # Compute corners of robot before rotation & translation
    corner1 = [rl / 2.0, rw / 2.0]
    corner2 = [rl / 2.0, -rw / 2.0]
    corner3 = [-rl / 2.0, rw / 2.0]
    corner4 = [-rl / 2.0, -rw / 2.0]
    corners = np.array([corner1, corner2, corner3, corner4]).transpose()

    # Rotation matrix
    R = rotation_matrix(theta, (0, 0, 1))[0:2, 0:2]

    # Compute new corner coords
    rot_corners = np.matmul(R, corners)

    # Translate to current position
    rot_corners[0, :] = rot_corners[0, :] + pose.position.x
    rot_corners[1, :] = pose.position.y - rot_corners[1, :]
    rot_corners = (rot_corners * scale).astype(np.int32)

    # Draw edges on map
    cv2.line(img, (rot_corners[0, 0], rot_corners[1, 0]), (rot_corners[0, 1], rot_corners[1, 1]), r_color, 2)
    cv2.line(img, (rot_corners[0, 1], rot_corners[1, 1]), (rot_corners[0, 3], rot_corners[1, 3]), r_color, 2)
    cv2.line(img, (rot_corners[0, 0], rot_corners[1, 0]), (rot_corners[0, 2], rot_corners[1, 2]), r_color, 2)
    cv2.line(img, (rot_corners[0, 2], rot_corners[1, 2]), (rot_corners[0, 3], rot_corners[1, 3]), r_color, 2)

    # Return adjusted image
    return img


# ------------------------------------------------------------------------- #
#
# This adds an arc to a provided 'base image' (which can be produced with
# 'generate_base_img') which shows where a robot with the provided pose will
# travel in the next 'tt' (total time) seconds if it continues with the
# specified twist. The granularity of the simulation can be adjusted with the
# 'dt' parameter, as well as the amount of time simulated, with 'tt.' Only
# the x component of the Twist's linear velocity and the z component of its
# angular velocity are considered. Scale determines the pixels / meter of
# the image
#
# ------------------------------------------------------------------------- #
def draw_arc(base_img, pose, twist, dt=0.1, tt=2.0, scale=100):

    # Copy image
    img = np.copy(base_img)

    # Get quaternion from pose message
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    # Convert quaternion rotation to rpy, extract yaw
    theta = euler_from_quaternion(quat)[2]

    # Draw arc based on current robot twist
    ct = 0.0
    prev_point = [pose.position.x, pose.position.y]
    prev_theta = theta

    while ct < tt:
        avg_theta = prev_theta + (0.5 * twist.angular.z * dt)
        new_x = prev_point[0] + (twist.linear.x * cos(avg_theta) * dt)
        new_y = prev_point[1] - (twist.linear.x * sin(avg_theta) * dt)

        # translated, scaled points for drawing
        prev_x_plot = int(prev_point[0] * scale)
        prev_y_plot = int(prev_point[1] * scale)
        new_x_plot = int(new_x * scale)
        new_y_plot = int(new_y * scale)

        # add arc segment to drawing
        cv2.line(img, (prev_x_plot, prev_y_plot), (new_x_plot, new_y_plot), (0, 0, 150), 2)

        # Update variables
        ct += dt
        prev_theta = prev_theta + (avg_theta - prev_theta) * 2
        prev_point = [new_x, new_y]

    # Return the adjusted image
    return img


# ------------------------------------------------------------------------- #
#
# An object for keeping track of the robot's current state. Subscribers
# should be assigned to the update functions for an object so that the class
# variables of a StateListener object are updated with real value from the
# robot. That way, values will be kept up to date and the latest values can
# be accessed at any time
#
# ------------------------------------------------------------------------- #
class StateListener:

    def __init__(self):

        # Robot initially motionless
        init_twist = Twist()
        init_twist.linear.x = 0.0
        init_twist.angular.z = 0.0

        # Variables to store data for the next image
        self.robot_pose = None
        self.robot_twist = init_twist
        self.robot_goal = None
        self.obstacles = None
        self.marker_status = False

    # Callback function to update robot pose
    def update_robot_pose(self, msg):
        self.robot_pose = msg

    # Callback function to update robot twist
    def update_robot_twist(self, msg):
        self.robot_twist = msg

    # Callback function to update robot goal pose
    def update_robot_goal(self, msg):
        self.robot_goal = msg

    # Callback function to update marker status
    def update_marker_status(self, msg):
        self.robot_goal = msg.data


# ------------------------------------------------------------------------- #
# THIS CODE WILL RUN WHEN THIS NODE IS LAUNCHED. PUBLISHES AN IMAGE SHOWING
# THE CURRENT POSE AND TRAJECTORY OF THE ROBOT, AS WELL AS THE ROBOT'S GOAL
# POSE
# ------------------------------------------------------------------------- #
if __name__ == '__main__':

    # Create handler to store values from updates
    data = StateListener()

    # Initialize as ROS node
    rospy.init_node('generate_data_vis_images')

    # Subscribe to topics
    robot_pose_sub = rospy.Subscriber('updates/robot_pose', Pose, data.update_robot_pose, queue_size=1)
    robot_twist_sub = rospy.Subscriber('updates/robot_twist', Twist, data.update_robot_twist, queue_size=1)
    robot_goal_sub = rospy.Subscriber('updates/goal_pose', Pose, data.update_robot_goal, queue_size=5)
    marker_status_sub = rospy.Subscriber('updates/marker_status', Bool, data.update_marker_status, queue_size=1)

    # Initialize publisher for image frames
    img_pub = rospy.Publisher('mc/data_vis_frames', CompressedImage, queue_size=1)

    # ROS stuff ready to go
    rospy.loginfo("Data Visualization Image Publisher initialized...")

    # Bridge for converting to ROS Image messages
    bridge = CvBridge()

    # Create base image
    img_scale = 150
    base = generate_base_img(scale=img_scale, bc='left')

    # Loop continuously
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        # Draw robot pose and arc if any data has been received
        if data.robot_pose:
            frame = draw_robot(base, data.robot_pose, scale=img_scale)
            frame = draw_arc(frame, data.robot_pose, data.robot_twist, scale=img_scale)
        else:
            frame = np.copy(base)

        # Add goal pose to image if one available
        if data.robot_goal:
            frame = draw_robot(frame, data.robot_goal, r_color=(200, 200, 100), p_color=(150, 150, 150), scale=img_scale)

        # Convert numpy array to ROS Image message
        img_msg = bridge.cv2_to_compressed_imgmsg(frame, 'png')

        # Publish message
        img_pub.publish(img_msg)

        # Sleep at loop rate
        rate.sleep()



