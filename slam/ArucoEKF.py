#!/usr/bin/env python

# Implements an EKF algorithm from the FilterPy library for the Aruco Marker data input

from filterpy.kalman import EKF
import numpy as np

# import ros packages
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


class ArucoExtendedKalmanFilter:

    def __init__(self):

        # Initialize the Extended Kalman Filter Object
        # First argument is dimension of the state vector X
        # For AruCo, our state vector consists of (x,y,theta)
        # Second arugment is dimension of the measurement input Z
        # For AruCo, measurement input is (x,y,theta)
        self.arucoEKF = EKF.ExtendedKalmanFilter(7,7)

        '''
        EKF.x initialized to zeros by constructor

        EKF.F = I; F = state transition matrix; state does not change w/o control input
        '''

        # Define the measurement noise covariance matrix
        self.arucoEKF.R = np.eye(7) * 1

        # Define the process noise covariance matrix
        #arucoEKF.Q = 

    def update(self,pose):
        #Initialize empty np array
        z = np.empty((7,1))

        #Convert Pose object into a 7x1 numpy array to pass to 
        z[0][1] = pose.position.x
        z[1][1] = pose.position.y
        z[2][1] = pose.position.z
        z[3][1] = pose.quaternion.x
        z[4][1] = pose.quaternion.y
        z[5][1] = pose.quaternion.z
        z[6][1] = pose.quaternion.w

        self.arucoEKF.predict_update(z, self.HJacobian, self.hx)

    def getPose(self):
        # pose = Pose()
        # pose.position.x = self.arucoEKF.x[0][1]
        return Pose()
    def getPoseCovStamped(self):
        return PoseWithCovarianceStamped()


    # Define the Jacobian matrix for measurement update
    # Robot movement in each direction is independent of the other two directions
    def HJacobian(self, x):
        return np.eye(7) 

    # Define callback function to calculate the expected sensor measurement
    # mean state vector is a distance; ArUco pose estimation algorithm measures distance
    # therefore our h(x) is just an identity matrix
    def hx(self, x):
        return x

# Run ROS Node
if(__name__ == "__main__"):
    # Initialize EKF for Aruco Markers
    arucoEKF = ArucoExtendedKalmanFilter()
    
    # Init ros node as aruco_ekf
    rospy.init_node("aruco_ekf")
    r = rospy.Rate(10) # 10hz

    rospy.Subscriber('aruco_pose_raw', Pose, arucoEKF.update)

    posePublisher = rospy.Publisher('aruco_pose_filtered', Pose, queue_size=1)
    poseCovStampPublisher = rospy.Publisher('aruco_pose_filtered_covariance', PoseWithCovarianceStamped, queue_size=1)

    while not rospy.is_shutdown():
        posePublisher.publish(arucoEKF.getPose())
        poseCovStampPublisher.publish(arucoEKF.getPoseCovStamped())
        r.sleep()

