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
from std_msgs.msg import Bool

# Import other required packages
from filterpy.kalman import EKF
import numpy as np
from math import acos, pi

from tf.transformations import euler_from_quaternion, quaternion_from_euler


class ArucoExtendedKalmanFilter:

    def __init__(self):

        # Initialize the Extended Kalman Filter Object
        # First argument is dimension of the state vector X
        # For AruCo, our state vector consists of (x,y,theta)      TODO: fix this - (x,y,z) for pos, (x,y,z,w) for orientation (a quaternion)
        # Second arugment is dimension of the measurement input Z
        # For AruCo, measurement input is (x,y,theta)

        self.arucoEKF = EKF.ExtendedKalmanFilter(6,6)

        '''
        EKF.x initialized to zeros by constructor

        EKF.F = I; F = state transition matrix; state does not change w/o control input
        '''

        # Define the measurement noise covariance matrix
        self.arucoEKF.R = np.eye(6) * np.array([0.05, 0.05, 0.1, 0.1, 0.1, 0.1])

        # Define the process noise covariance matrix
        #arucoEKF.Q =

        self.acuroDetected = False
        self.first_pass = True

        self.last_orientation = None


    def arucoDetected(self, msg):
        self.arucoDetected = msg.data


    def check_distance(self, new_orientation):

        # If this is the first time, not a bad value
        if self.last_orientation is None:
            self.last_orientation = new_orientation
            return False

        # Otherwise, get angle between new and old orientation
        dp = np.dot(np.array(self.last_orientation), np.array(new_orientation))
        angle = acos(dp) * 2

        self.last_orientation = new_orientation
        return angle > (pi / 20.0)



    def update(self,pose):

        # Array to store new info in
        z = np.zeros(shape=(6, 1))

        # Get translation information
        z[0, 0] = pose.position.x
        z[1, 0] = pose.position.y
        z[2, 0] = pose.position.z

        # Get quaternion
        quaternion = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        # Get convert pose from quaternion to euler
        rpy = euler_from_quaternion(quaternion)

        # Add rpy to z
        z[3, 0] = rpy[0]
        z[4, 0] = rpy[1]
        z[5, 0] = rpy[2]

        # Check if angle change exceeds threshold
        bad_value = False # lf.check_distance(quaternion)

        if self.arucoDetected and not bad_value:
            self.arucoEKF.predict_update(z, self.HJacobian, self.hx)


    def getPose(self):

        # Create pose message
        pose = Pose()

        # Fill position values
        pose.position.x = self.arucoEKF.x[0][0]
        pose.position.y = self.arucoEKF.x[1][0]
        pose.position.z = self.arucoEKF.x[2][0]

        # Convert rpy to quaternion
        quaternion = quaternion_from_euler(
            self.arucoEKF.x[3][0],
            self.arucoEKF.x[4][0],
            self.arucoEKF.x[5][0]
        )

        # Add quaternion to message
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    def getPoseCovStamped(self):

        poseCovStamped = PoseWithCovarianceStamped()
        poseCovStamped.header.frame_id = str("world")
        poseCovStamped.pose.pose = self.getPose()
        return poseCovStamped


    # Define the Jacobian matrix for measurement update
    # Robot movement in each direction is independent of the other two directions
    def HJacobian(self, x):
        return np.eye(6)

    # Define callback function to calculate the expected sensor measurement
    # mean state vector is a distance; ArUco pose estimation algorithm measures distance
    # therefore our h(x) is just an identity matrix
    def hx(self, x):
        return x


# Run ROS Node
if(__name__ == "__main__"):

    # Initialize EKF for Aruco Markers
    robot_pose_ekf = ArucoExtendedKalmanFilter()
    
    # Init ros node as aruco_ekf
    rospy.init_node("robot_pose_ekf")

    # Read unfiltered pose topic from parameter server
    unfiltered_pose_topic = rospy.get_param("unfiltered_pose_topic")

    # Create subscriber to pose updates from aruco node
    poseSubscriber = rospy.Subscriber(
        unfiltered_pose_topic, 
        Pose, 
        robot_pose_ekf.update
    )

    # subscribe to the marker_detected topic
    markerDetectedSubscriber = rospy.Subscriber(
        'aruco/marker_detected', 
        Bool, 
        robot_pose_ekf.arucoDetected
    )

    # Create publishers for filtered pose messages
    posePublisher = rospy.Publisher('ekf/pose_filtered', Pose, queue_size=1)
    poseCovStampPublisher = rospy.Publisher(
        'ekf/pose_filtered_covariance', 
        PoseWithCovarianceStamped, 
        queue_size=1
    )

    # Set loop rate
    r = rospy.Rate(20)  # 10hz

    while not rospy.is_shutdown():
        posePublisher.publish(robot_pose_ekf.getPose())
        poseCovStampPublisher.publish(robot_pose_ekf.getPoseCovStamped())
        r.sleep()