#!/usr/bin/env python

# ----------------------------- #
# Subscribes to drive command, Publishes custom motor speeds. #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
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
        CONST_ROBOT_WIDTH = 0.75
        CONST_WHEEL_DIAMETER = 0.15

        #FIXME: Calculations for motor speed

        speed = 0.5 * CONST_ROBOT_WIDTH * self.cmd_sub.angular
        speed = self.cmd_sub.linear + speed
        speed = speed / (CONST_WHEEL_DIAMETER / 2)
        print(speed)
        return speed

        #publish the motor speeds
        motor_speeds = DriveMotorSpeeds()

        #determine which side is positive
        left_side = -1
        right_side = 1
        motor_speeds.front_l = speed * left_side
        motor_speeds.rear_l = speeds * left_side
        motor_speeds.front_r = speed * right_side
        motor_speeds.rear_r = speed * right_side

        self.speeds_pub.Publish(motor_speeds)
        pass

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