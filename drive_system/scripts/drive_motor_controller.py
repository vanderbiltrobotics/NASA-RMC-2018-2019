#!/usr/bin/env python

# ----------------------------- #
# Subscribes to drive command, Publishes custom motor speeds. #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from custom_msgs.msg import DriveMotorSpeeds

class DriveController:

    # Constructor
    def __init__(self):
        # Initialize publishers
        self.speeds_pub = rospy.Publisher('drive_motor_speeds', DriveMotorSpeeds, queue_size=1)

        # Initialize subscribers
        self.cmd_sub = rospy.Subscriber('drive_cmd', Twist, self.process_drive_cmd)

    # Callback function for new drive commands
    def process_drive_cmd(self, data):
        linear = data.linear.x
        angular = data.angular.x

        # Constant robot values - will be in parameter server
        CONST_ROBOT_WIDTH = 0.75
        CONST_WHEEL_DIAMETER = 0.1524  # in meters

        # Compute linear and angular velocity components
        linear_component = (linear / CONST_WHEEL_DIAMETER) / 2
        angular_component = (angular * CONST_ROBOT_WIDTH / 2) / (CONST_WHEEL_DIAMETER / 2)

        # Clockwise is negative, Counter-Clockwise is positive
        right_speeds = linear_component - angular_component
        left_speeds = linear_component + angular_component

        # publish the motor speeds
        motor_speeds = DriveMotorSpeeds()

        # Assign and publish motor speed messages
        motor_speeds.front_l = left_speeds
        motor_speeds.rear_l = left_speeds
        motor_speeds.front_r = right_speeds
        motor_speeds.rear_r = right_speeds

        self.speeds_pub.publish(motor_speeds)


# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('drive_motor_controller')

    # Create a DriveController object
    controller = DriveController()

    # Ready to go
    rospy.loginfo("Drive Motor Controller initialized...")

    # Loop continuously
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pass
