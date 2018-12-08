#!/usr/bin/env python

# ----------------------------- #
# DESCRIPTION OF THIS NODE HERE #
# ----------------------------- #

# import required packages
import rospy
from std_msgs.msg import UInt8

# NOT SURE HOW THIS NODE SHOULD BE IMPLEMENTED YET

def callback(msg):
	pass


# Run the node
if __name__ == '__main__':

	# Initialize as ROS node
	rospy.init_node('can_bus_interface')
    
    # Create subscribers to the relevant topics
    rospy.Subscriber('dig_motor_speeds', UInt8, callback)
    rospy.Subscriber('drive_motor_speeds', UInt8, callback)

    # Ready to go
    rospy.loginfo("CAN Bus Interface initialized...")