#!/usr/bin/env python

# Implements an EKF algorithm from the FilterPy library for the Aruco Marker data input

from filterpy.kalman import EKF
import numpy as np

# import ros packages
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

# Define the Jacobian matrix for measurement update
# Robot movement in each direction is independent of the other two directions
def HJacobian(x):
    return np.eye(7) 

# Define callback function to calculate the expected sensor measurement
# mean state vector is a distance; ArUco pose estimation algorithm measures distance
# therefore our h(x) is just an identity matrix
def hx(x):
    return x


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

    def update(self,z):
        self.arucoEKF.predict_update(z,HJacobian,hx)
