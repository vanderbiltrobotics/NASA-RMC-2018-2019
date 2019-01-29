#!/usr/bin/env python

# ----------------------------- #
# DESCRIPTION OF THIS NODE HERE #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopControl:
    
    # Constructor
    def __init__(self):
        
        # Initialize publishers
        self.drive_pub = rospy.Publisher('drive_cmd', Twist)

        # Initialize subscribers
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback)


    # Callback function for joystick controls
    def joy_callback(self, data):
        twist = Twist()
        twist.linear.x = data.axes[1]
        twist.angular.z = data.axes[0]
        self.drive_pub.publish(twist)


if __name__ == '__main__':
    
    # Initialize as ROS node
    rospy.init_node('teleop_control')

    # Create a TeleopControl object 
    control = TeleopControl()

    # Ready to go
    rospy.loginfo("Teleop Control initialized...")

    # Loop continuously
    while not rospy.is_shutdown():
    	pass