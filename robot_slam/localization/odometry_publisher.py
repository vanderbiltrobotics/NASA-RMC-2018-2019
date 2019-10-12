#!/usr/bin/env python

#This script publishes odometry messages from aruco EKF pose data

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Header


class OdometryPub:

    def __init__(self):
        self.pub = rospy.Publisher('test_odom', Odometry, queue_size=0)
        self.header = 0
        self.msg = 0
        self.data = 0

    def publishmsg(self):
        self.msg = Odometry(header=self.header,pose = self.data)
        self.pub.publish(self.msg)

    def posewithcov(self, PoseWithCov):
        self.data = PoseWithCov.pose
        self.data.orientation.x = 0
        self.data.orientation.z = 0
        self.data.orientation.w = 0.7071068
        self.data.orientation.y = 0.7071068
        self.header = PoseWithCov.header

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('odometry_publisher')

    odo = OdometryPub()

    sub = rospy.Subscriber('ekf/pose_filtered_covariance', PoseWithCovarianceStamped, odo.posewithcov)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        odo.publishmsg()
        rate.sleep()
