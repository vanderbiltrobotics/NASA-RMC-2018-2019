#!/usr/bin/env python

# ----------------------------------------------- #
# Table of index number of /joy.buttons:
# Index   Button name on the actual controller
#
# 0   A
# 1   B
# 2   X
# 3   Y
# 4   LB
# 5   RB
# 6   back
# 7   start
# 8   power
# 9   Button stick left
# 10  Button stick right

# Table of index number of /joy.axes:
# Index   Axis name on the actual controller
#
# 0   Moving left joystick left (+) and right (-) changes rotational velocity
# 1   Moving left joystick up (+) and down (-) changes linear velocity
# 2   LT
# 3   Left/Right Axis stick right
# 4   Up/Down Axis stick right
# 5   RT
# 6   cross key left/right
# 7   cross key up/down
# ----------------------------------------------- #

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