#!/usr/bin/env python

# Tests the effectiveness of an EKF on ArUco Board pose data. Subscribes to both raw ArUco pose
# data and EKF data and compares their standard deviation and covariance.

from filterpy.kalman import EKF
import numpy as np

# import ros packages
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped


class ArucoPoseTracker:

    def __init__(self):

        # Indices to track how many data points we have read in out of the total
        self.indexRaw = 0
        self.indexEKF = 0

        # Store raw and filtered aruco pose data
        self.arucoRaw = np.zeros((7,total_data_points))
        self.arucoEKF = np.zeros((7,total_data_points))
        self.arucoEKFCov = np.zeros((6,6))

        # Store mean and standard deviation of all dimensions. 
        # The mean is stored in column 1 and the standard deviation in column 2.
        self.arucoRawMeanAndStd = np.zeros((7,2))
        self.arucoEKFMeanandStd = np.zeros(7,2))

    # Record raw aruco data
    def exportRaw(self,pose):
        
        # Store raw data in the next open array index
        self.arucoRaw[0][self.indexRaw] = pose.position.x 
        self.arucoRaw[1][self.indexRaw] = pose.position.y 
        self.arucoRaw[2][self.indexRaw] = pose.position.z 
        self.arucoRaw[3][self.indexRaw] = pose.orientation.x  
        self.arucoRaw[4][self.indexRaw] = pose.orientation.y 
        self.arucoRaw[5][self.indexRaw] = pose.orientation.z 
        self.arucoRaw[6][self.indexRaw] = pose.orientation.w  

        # Increment raw index
        self.indexRaw += 1

    # Record ekf aruco data
    def exportFiiltered(self,poseWithCovariance):

        # Store ekf pose data in the next open array index
        self.arucoEKF[0][self.indexEKF] = poseWithCovariance.pose.pose.x 
        self.arucoEKF[1][self.indexEKF] = poseWithCovariance.pose.pose.y
        self.arucoEKF[2][self.indexEKF] = poseWithCovariance.pose.pose.z 
        self.arucoEKF[3][self.indexEKF] = poseWithCovariance.pose.orientation.x 
        self.arucoEKF[4][self.indexEKF] = poseWithCovariance.pose.orientation.y 
        self.arucoEKF[5][self.indexEKF] = poseWithCovariance.pose.orientation.z 
        self.arucoEKF[6][self.indexEKF] = poseWithCovariance.pose.orientation.w 

        # Store ekf covariance data in the next open array index
        self.arucoEKFCov = poseWithCovariance.pose.covariance

        # Increment EKF index
        self.indexEKF += 1

    # Calculate mean and standard deviation 
    def calculateMeanAndStd(self):

        # Calculates the raw and ekf mean and standard deviation in all dimensions
        for x in range(7):
            self.arucoRawMeanAndStd[x][0] = np.mean(self.arucoRaw[x])
            self.arucoRawMeanAndStd[x][1] = np.std(self.arucoRaw[x])
            self.arucoEKFMeanAndStd[x][0] = np.mean(self.arucoEKF[x])
            self.arucoEKFMeanAndStd[x][1] = np.std(self.arucoEKF[x])

            


        

# Run ROS Node
if(__name__ == "__main__"):

    # Init ros node as test_ekf
    rospy.init_node("test_ekf")

    total_data_points = rospy.get_param("total_data_points")
    true_x = rospy.get_param("true_x")
    true_y = rospy.get_param("true_y")
    true_z = rospy.get_param("true_z")
    true_orientation_x = rospy.get_param("true_orientation_x")
    true_orientation_y = rospy.get_param("true_orientation_y")
    true_orientation_z = rospy.get_param("true_orientation_z")
    true_orientation_w = rospy.get_param("true_orientation_w")

    # Initialize data capture for aruco pose data
    aruco_pose_tracker = ArucoPoseTracker()

    while aruco_pose_tracker.indexRaw <= total_data_points and
            aruco_pose_tracker.indexEKF <= total_data_points:

        poseSubscriber_raw = rospy.Subscriber('aruco_pose_raw', Pose, aruco_pose_tracker.exportRaw)
        poseSubscriber_filter_cov = rospy.Subscriber('aruco_pose_filtered_covariance', PoseWithCovarianceStamped,exportFiltered)

        rospy.spin()

    # Calculate mean and standard deviation once done collecting data. Print to console.
