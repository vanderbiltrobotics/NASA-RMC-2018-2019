#!/usr/bin/env python
import rospy
from [MESSAGE_TYPE_PACKAGE] import [MESSAGE_TYPE]

class Handler:
    def __init__(self):
        self.dig_motor_speeds_publisher = rospy.Publisher('dig_motor_speeds', [MESSAGE_TYPE], [QUEUE_TIME])
        self.dig_feedback_publisher = rospy.Publisher('dig_feedback', [MESSAGE_TYPE], [QUEUE_TIME])
    def callback(data):
        pass

def communicate():
    rospy.init_node('dig_motor_controller', anonymous=True)
    h = Handler()
    rospy.Subscriber('dig_cmd', [MESSAGE_TYPE], h.callback)