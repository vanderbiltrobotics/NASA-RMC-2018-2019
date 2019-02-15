#!/usr/bin/env python

# ----------------------------- #
# Subscribes to drive command, Publishes custom motor speeds. #
# ----------------------------- #

# Import ROS packages
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Import other required packages
from math import sin, cos, pi
import numpy as np

class Robot:

    def __init__(self, twist_topic, pose_topic, init_pose):

        # True state of robot
        self.pose = init_pose
        self.desired_lin_vel = 0.0
        self.desired_ang_vel = 0.0

        # Subscriber to listen for incoming drive commands
        self.twist_sub = rospy.Subscriber(twist_topic, Twist, self.set_twist)

        # Publisher to send pose estimates
        self.pose_pub = rospy.Publisher(pose_topic, Pose, queue_size=0)
        self.pose_stamped_pub = rospy.Publisher(pose_topic + "_stamped", PoseStamped, queue_size=0)

    # Sets the robot's desired velocities from the incoming message
    def set_twist(self, new_twist_msg):
        self.desired_lin_vel = new_twist_msg.linear.x
        self.desired_ang_vel = new_twist_msg.angular.z

    # Computes new pose after moving at current twist for the specified time
    def move_for_time(self, sec):

        # Compute actual velocities for this time-step
        noisy_lin_vel = self.desired_lin_vel + np.random.normal(0.0, abs(self.desired_lin_vel / 5.0))
        noisy_ang_vel = self.desired_ang_vel + np.random.normal(0.0, abs(self.desired_ang_vel / 5.0))

        # Convert orientation to rpy
        quat = (
            self.pose.pose.orientation.x,
            self.pose.pose.orientation.y,
            self.pose.pose.orientation.z,
            self.pose.pose.orientation.w
        )
        rpy = euler_from_quaternion(quat)

        # Compute new position and orientation
        init_theta = rpy[2]
        final_theta = init_theta + (noisy_ang_vel * sec)
        avg_theta = (final_theta + init_theta) / 2.0
        final_x = self.pose.pose.position.x + (noisy_lin_vel * cos(avg_theta) * sec)
        final_y = self.pose.pose.position.y + (noisy_lin_vel * sin(avg_theta) * sec)

        # Limit final_theta to range (-pi, pi)
        final_theta = final_theta - (2 * pi) if final_theta > pi else final_theta
        final_theta = final_theta + (2 * pi) if final_theta < -pi else final_theta

        # Create quaternion from theta
        r = 0.0
        p = 0.0
        y = final_theta
        quat = quaternion_from_euler(r, p, y)

        # Update current pose with new values
        self.pose.pose.position.x = final_x
        self.pose.pose.position.y = final_y
        self.pose.pose.orientation.x = quat[0]
        self.pose.pose.orientation.y = quat[1]
        self.pose.pose.orientation.z = quat[2]
        self.pose.pose.orientation.w = quat[3]


    # Publish estimate of the robot's pose (if noise is set, won't exactly match true pose)
    def send_pose_est(self):
        self.pose_pub.publish(self.pose.pose)
        self.pose_stamped_pub.publish(self.pose)



if __name__ == "__main__":

    # Create base image for viewing simulation
    field_width = 7.0
    field_length = 3.0
    scale = 150
    base_img = np.zeros((int(field_length * scale), int(field_width * scale), 3), np.uint8) + 244
    p_color = (0, 150, 0)
    r_color = (150, 0, 0)

    # Initialize ROS node
    rospy.init_node("path_tracking_simulator")

    # Initial pose for robot
    init_pose = PoseStamped()
    init_pose.header.frame_id = "world"
    init_pose.pose.position.x = field_width / 2.0 - 2.0
    init_pose.pose.position.y = field_length / 2.0 - 0.75
    init_quat = quaternion_from_euler(0, 0, -1.0)
    init_pose.pose.orientation.x = init_quat[0]
    init_pose.pose.orientation.y = init_quat[1]
    init_pose.pose.orientation.z = init_quat[2]
    init_pose.pose.orientation.w = init_quat[3]

    # Create a simulated robot
    robot = Robot("cmd_vel", "cur_pose", init_pose)

    # Create example path
    test_path = Path()
    test_path.header.frame_id = "world"
    for i in range(500):
        new_pose = PoseStamped()
        new_pose.header.frame_id = "world"
        new_pose.pose.position.x = 1.0 + i * 0.01
        new_pose.pose.position.y = 1.5 + 0.20 * sin(2.0 * pi * (1.0 / 50.0) * i)
        test_path.poses.append(new_pose)

    # Create path publisher
    path_pub = rospy.Publisher("cur_path", Path, queue_size=0)

    # Wait for connections, then publish
    rospy.sleep(2)
    path_pub.publish(test_path)

    # Set update rate for robots (how often path tracking node can receive new pose estimate)
    loop_frequency = 100
    update_rate = rospy.Rate(loop_frequency)

    # Run simulation
    while not rospy.is_shutdown():

        # Move robot
        robot.move_for_time(1.0 / loop_frequency)

        # Send pose estimate
        robot.send_pose_est()

        # Sleep to maintain loop rate
        update_rate.sleep()


