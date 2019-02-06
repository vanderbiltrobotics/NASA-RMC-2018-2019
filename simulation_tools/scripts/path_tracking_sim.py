#!/usr/bin/env python

# ----------------------------- #
# Subscribes to drive command, Publishes custom motor speeds. #
# ----------------------------- #

# Import ROS packages
import rospy
from geometry_msgs.msg import Twist, Pose

# Import other required packages
from math import sin, cos, pi
import numpy as np
import cv2

class Robot:

    def __init__(self, twist_topic, pose_topic, init_pose=Pose()):

        # True state of robot
        self.pose = init_pose
        self.desired_lin_vel = 1.0
        self.desired_ang_vel = 2 / -2.12132034356

        # Subscriber to listen for incoming drive commands
        self.twist_sub = rospy.Subscriber(twist_topic, Twist, self.set_twist)

        # Publisher to send pose estimates
        self.pose_pub = rospy.Publisher(pose_topic, Pose, queue_size=0)

    # Sets the robot's desired velocities from the incoming message
    def set_twist(self, new_twist_msg):
        self.desired_lin_vel = new_twist_msg.linear.x
        self.desired_ang_vel = new_twist_msg.angular.x

    # Computes new pose after moving at current twist for the specified time
    def move_for_time(self, sec):

        # Compute actual velocities for this time-step
        noisy_lin_vel = self.desired_lin_vel # + noise
        noisy_ang_vel = self.desired_ang_vel # + noise

        # Compute new position and orientation
        init_theta = self.pose.orientation.x
        final_theta = self.pose.orientation.x + (noisy_ang_vel * sec)
        avg_theta = (final_theta + init_theta) / 2.0
        final_x = self.pose.position.x + (noisy_lin_vel * cos(avg_theta) * sec)
        final_y = self.pose.position.y + (noisy_lin_vel * sin(avg_theta) * sec)

        # Limit final_theta to range (-pi, pi)
        final_theta = final_theta - (2 * pi) if final_theta > pi else final_theta
        final_theta = final_theta + (2 * pi) if final_theta < -pi else final_theta

        # Update current pose with new values
        self.pose.position.x = final_x
        self.pose.position.y = final_y
        self.pose.orientation.x = final_theta


    # Publish estimate of the robot's pose (if noise is set, won't exactly match true pose)
    def send_pose_est(self):
        self.pose_pub.publish(self.pose)




if __name__ == "__main__":

    # Create base image for viewing simulation
    field_width = 5.0
    field_length = 5.0
    scale = 150
    base_img = np.zeros((int(field_length * scale), int(field_width * scale), 3), np.uint8) + 244
    p_color = (0, 150, 0)
    r_color = (150, 0, 0)

    # Initialize ROS node
    rospy.init_node("path_tracking_simulator")

    # Initial pose for robot
    init_pose = Pose()
    init_pose.position.x = 0.5
    init_pose.position.y = 0.5
    init_pose.orientation.x = pi / 4

    # Create a simulated robot
    robot = Robot("drive_cmd", "current_pose_estimate", init_pose)

    # Set update rate for robots (how often path tracking node can receive new pose estimate)
    loop_frequency = 100
    update_rate = rospy.Rate(loop_frequency)

    # Run simulation
    while not rospy.is_shutdown():

        # Move robot
        robot.move_for_time(1.0 / loop_frequency)

        # Send pose estimate
        robot.send_pose_est()

        # Copy the base image, draw the robot in it's current position
        frame = np.copy(base_img)

        # Get angle information from current pose
        theta = robot.pose.orientation.x

        # Draw arrow representing robot pose
        arrow_x1 = robot.pose.position.x
        arrow_y1 = field_length - robot.pose.position.y   # subtract so increased y = up in image
        arrow_x2 = arrow_x1 + (cos(theta) * .15)
        arrow_y2 = arrow_y1 - (sin(theta) * .15)
        cv2.arrowedLine(frame, (int(arrow_x1 * scale), int(arrow_y1 * scale)),
                        (int(arrow_x2 * scale), int(arrow_y2 * scale)), p_color, 2, tipLength=0.5)
        cv2.circle(frame, (int(arrow_x1 * scale), int(arrow_y1 * scale)), 5, r_color, thickness=-1)

        # DRAW TEST POINT
        target_point = [1.7071067811865475, 0.7071067811865475]
        cv2.circle(frame, (int(target_point[0] * scale), int((field_length - target_point[1]) * scale)), 5, (0, 0, 255), thickness=-1)

        # Display image
        cv2.imshow("Path tracking simulation", frame)
        cv2.waitKey(1)

        # Sleep to maintain loop rate
        update_rate.sleep()


