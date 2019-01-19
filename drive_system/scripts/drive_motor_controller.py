#!/usr/bin/env python

# ----------------------------- #
# DESCRIPTION OF THIS NODE HERE #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


class DriveController:

	# Constructor
    def __init__(self):

    	# Initialize publishers
        self.speeds_pub = rospy.Publisher('drive_motor_speeds', UInt8, queue_size=1)

        # Initialize subscribers
        self.cmd_sub = rospy.Subscriber('drive_cmd', Twist, self.process_drive_cmd)

        #FIXME: update params
        self.get_wheel_ang_vel(10, 10, 1)


    # Callback function for new drive commands
    def process_drive_cmd(self, data):
        pass

    @staticmethod
    def get_wheel_ang_vel(lin_vel, ang_vel, side):

        CONST_ROBOT_WIDTH = 0.75
        CONST_WHEEL_DIAMETER = 0.15

        if side != -1 and side != 1:
            raise ValueError("There is an invalid side value.")

        else:
            out = 0.5 * CONST_ROBOT_WIDTH * ang_vel
            out = lin_vel + (side * out)
            out = out / (CONST_WHEEL_DIAMETER / 2)
            print(out)
            return out


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