#!/usr/bin/env python
import rospy
from [MESSAGE_TYPE_PACKAGE] import [MESSAGE_TYPE]

class Handler:
    def __init__(self):

    def callback(data):
        pass

def communicate():
    rospy.init_node('route_planning_execution', anonymous=True)
    h = Handler()
    rospy.Subscriber('goal_pose', [MESSAGE_TYPE], [CALLBACK])
    rospy.Subscriber('SLAM_obs', [MESSAGE_TYPE], [CALLBACK])
    rospy.Subscriber('SLAM_pose_est', [MESSAGE_TYPE], [CALLBACK])
    rospy.Subscriber('SLAM_twist_est', [MESSAGE_TYPE], [CALLBACK])
    rospy.Subscriber('imm_obs', [MESSAGE_TYPE], [CALLBACK])
    rospy.Publisher('drive_cmd', [MESSAGE_TYPE], [QUEUE_TIME])