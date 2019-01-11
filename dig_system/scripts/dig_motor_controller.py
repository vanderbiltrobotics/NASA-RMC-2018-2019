#!/usr/bin/env python

# ----------------------------- #
# DESCRIPTION OF THIS NODE HERE #
# ----------------------------- #

# import required packages
import rospy
from std_msgs.msg import UInt8


class DigController:

	# Constructor
    def __init__(self):

    	# Initialize current control setting to  0
    	self.current_mode = 0

    	# Initialize publishers
        self.speeds_pub = rospy.Publisher('dig_motor_speeds', UInt8, queue_size=1)
        self.feedback_pub = rospy.Publisher('dig_feedback', UInt8, queue_size=1)

        # Initialize subscriber
        self.cmd_sub = rospy.Subscriber('dig_cmd', UInt8, self.process_dig_cmd)

    # Callback function for dig_cmd subscribers
    # Changes the current mode and activates the new mode
    def process_dig_cmd(self, msg):

    	# Update the current mode
    	self.current_mode = msg.data
        #this is a random comment
        #TODO: delete comment
        


# Run the node
if __name__ == '__main__':

	# Initialize as ROS node
    rospy.init_node('dig_motor_controller')

    # Create a DigController object 
    controller = DigController()

    # Ready to go
    rospy.loginfo("Dig Motor Controller initialized...")

    # Loop continuously
    while not rospy.is_shutdown():
    	pass