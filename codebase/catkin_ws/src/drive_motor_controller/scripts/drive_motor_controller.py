#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Handler:
    def __init__(self):
        self.publisher = rospy.Publisher('drive_motor_speeds', ,) #vector 4
    def callback(data):
        pass

def listener():
    rospy.init_node('drive_motor_controller', anonymous=True)
    h = Handler()
    rospy.Subscriber('drive_cmd', Twist, h.callback)