#!/usr/bin/env python
import rospy
from [MESSAGE_TYPE_PACKAGE] import [MESSAGE_TYPE]

class Handler:
    def __init__(self):
        #CAN bus
    def callback(data):
        pass

def communicate():
    rospy.init_node('can_bus_interface', anonymous=True)
    h = Handler()
    rospy.Subscriber('dig_motor_speeds', [MESSAGE_TYPE], h.callback)
    rospy.Subscriber('drive_motor_speeds', [MESSAGE_TYPE], h.callback)