#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Handler:
    def __init__(self):
        self.drive_motor_speeds_publisher = rospy.Publisher('drive_motor_speeds', [MESSAGE_TYPE], [QUEUE_TIME])
    def callback(data):
        pass

def communicate():
    rospy.init_node('drive_motor_controller', anonymous=True)
    h = Handler()
    rospy.Subscriber('drive_cmd', Twist, h.callback)