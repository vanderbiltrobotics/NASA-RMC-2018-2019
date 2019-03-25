#!/usr/bin/env python

# ------------------------------------------------------------------------------------- #
#
# EKF LOCALIZATION NODE
#
# Implements an EKF algorithm from the FilterPy library for the Aruco Marker data input
#
# ------------------------------------------------------------------------------------- #


# Import ROS packages
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

# Import other required packages
from filterpy.kalman import EKF
import numpy as np


class ArucoExtendedKalmanFilter:


    def __init__(self):

        # Initialize the Extended Kalman Filter Object
        # First argument is dimension of the state vector X
        # For AruCo, our state vector consists of (x,y,theta)      TODO: fix this - (x,y,z) for pos, (x,y,z,w) for orientation (a quaternion)
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

        # Consecutively increasing ID for header in pose with covariance stamped
        self.seqID = 0

    def update(self,pose):
        #Initialize empty np array
        z = np.empty((7,1))

        #Convert Pose object into a 7x1 numpy array to pass to 
        z[0][0] = pose.position.x
        z[1][0] = pose.position.y
        z[2][0] = pose.position.z
        z[3][0] = pose.orientation.x
        z[4][0] = pose.orientation.y
        z[5][0] = pose.orientation.z
        z[6][0] = pose.orientation.w

        self.arucoEKF.predict_update(z, self.HJacobian, self.hx)

    def getPose(self):
        pose = Pose()
        pose.position.x = 0.0 #self.arucoEKF.x[0][0]
        pose.position.y = 0.0 #self.arucoEKF.x[1][0]
        pose.position.z = self.arucoEKF.x[2][0]
        pose.orientation.x = self.arucoEKF.x[3][0]
        pose.orientation.y = self.arucoEKF.x[4][0]
        pose.orientation.z = self.arucoEKF.x[5][0]
        pose.orientation.w = 1.0 #self.arucoEKF.x[6][0]

        return pose

    def getPoseCovStamped(self):
        poseCovStamped = PoseWithCovarianceStamped()
        
        poseCovStamped.header.seq = self.seqID
        poseCovStamped.header.stamp = rospy.Time.now()
        poseCovStamped.header.frame_id = str(self.seqID)
        self.seqID += 1 
        
        poseCovStamped.pose.pose = self.getPose()
        poseCovStamped.pose.covariance = np.reshape(np.eye(6)*5, (36,)).tolist()
       
        return poseCovStamped


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

    # Create subscriber to pose updates from aruco node
    poseSubscriber = rospy.Subscriber('aruco/pose_raw', Pose, arucoEKF.update)

    # Create publishers for filtered pose messages
    posePublisher = rospy.Publisher('ekf/pose_filtered', Pose, queue_size=1)
    poseCovStampPublisher = rospy.Publisher('ekf/pose_filtered_covariance', PoseWithCovarianceStamped, queue_size=1)

    # Set loop rate
    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        posePublisher.publish(arucoEKF.getPose())
        poseCovStampPublisher.publish(arucoEKF.getPoseCovStamped())
        r.sleep()