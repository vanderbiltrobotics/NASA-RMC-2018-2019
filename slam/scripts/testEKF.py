#!/usr/bin/env python

# Tests the effectiveness of an EKF on ArUco Board pose data. Subscribes to both raw ArUco pose
# data and EKF data and compares their standard deviation and covariance.

# import ros packages
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

# Import other required packages
import numpy as np


class ArucoPoseTracker:

    def __init__(self,trueX,trueY,trueZ,
                trueOrientationX,trueOrientationY,trueOrientationZ,trueOrientationW,totalDataPoints):

        # True pose values 
        self.trueX = trueX
        self.trueY = trueY
        self.trueZ = trueZ
        self.trueOrientationX = trueOrientationX
        self.trueOrientationY = trueOrientationY
        self.trueOrientationZ = trueOrientationZ
        self.trueOrientationW = trueOrientationW
        self.totalDataPoints = totalDataPoints

        # Indices to track how many data points we have read in out of the total
        self.indexRaw = 0
        self.indexEKF = 0

        # Store raw and filtered aruco pose error
        self.arucoRaw = np.zeros((7, self.totalDataPoints))
        self.arucoEKF = np.zeros((7, self.totalDataPoints))

        # Store mean and standard deviation of all dimensions. 
        # The mean is stored in column 1 and the standard deviation in column 2.
        self.arucoRawMeanAndStd = np.zeros((7,2))
        self.arucoEKFMeanAndStd = np.zeros((7,2))

    # Record raw aruco error
    def exportRaw(self,pose):

        # Collect up to specified number of data points
        if self.indexRaw < self.totalDataPoints:

            # Store raw error in the next open array index
            self.arucoRaw[0][self.indexRaw] = pose.position.x - self.trueX
            self.arucoRaw[1][self.indexRaw] = pose.position.y - self.trueY
            self.arucoRaw[2][self.indexRaw] = pose.position.z - self.trueZ
            self.arucoRaw[3][self.indexRaw] = pose.orientation.x - self.trueOrientationX
            self.arucoRaw[4][self.indexRaw] = pose.orientation.y - self.trueOrientationY
            self.arucoRaw[5][self.indexRaw] = pose.orientation.z - self.trueOrientationZ
            self.arucoRaw[6][self.indexRaw] = pose.orientation.w - self.trueOrientationW

            # Increment raw index
            self.indexRaw += 1

    # Record ekf aruco error
    def exportFiltered(self,pose):

        # Collect up to specified number of data points
        if self.indexEKF < self.totalDataPoints:

            # Store ekf pose error in the next open array index
            self.arucoEKF[0][self.indexEKF] = pose.position.x - self.trueX
            self.arucoEKF[1][self.indexEKF] = pose.position.y - self.trueY
            self.arucoEKF[2][self.indexEKF] = pose.position.z - self.trueZ
            self.arucoEKF[3][self.indexEKF] = pose.orientation.x - self.trueOrientationX
            self.arucoEKF[4][self.indexEKF] = pose.orientation.y - self.trueOrientationY
            self.arucoEKF[5][self.indexEKF] = pose.orientation.z - self.trueOrientationZ
            self.arucoEKF[6][self.indexEKF] = pose.orientation.w - self.trueOrientationW

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

        return self.arucoRawMeanAndStd, self.arucoEKFMeanAndStd
        

# Run ROS Node
if __name__ == "__main__":

    # Init ros node as test_ekf
    rospy.init_node("test_ekf")

    # Read parameters from server
    totalDataPoints = rospy.get_param("total_data_points")
    trueX = rospy.get_param("true_x")
    trueY = rospy.get_param("true_y")
    trueZ = rospy.get_param("true_z")
    trueOrientationX = rospy.get_param("true_orientation_x")
    trueOrientationY = rospy.get_param("true_orientation_y")
    trueOrientationZ = rospy.get_param("true_orientation_z")
    trueOrientationW = rospy.get_param("true_orientation_w")

    # Initialize data capture for aruco pose data
    aruco_pose_tracker = ArucoPoseTracker(trueX,trueY,trueZ,
                        trueOrientationX,trueOrientationY,trueOrientationZ,trueOrientationW,totalDataPoints)

    # Create subscribers to raw and filtered pose topics
    poseSubscriber_raw = rospy.Subscriber('aruco/pose_raw', Pose, aruco_pose_tracker.exportRaw)
    poseSubscriber_filter_cov = rospy.Subscriber('ekf/pose_filtered', Pose, aruco_pose_tracker.exportFiltered)

    # Wait until enough data points have been collected
    while aruco_pose_tracker.indexRaw < totalDataPoints and \
            aruco_pose_tracker.indexEKF < totalDataPoints and not rospy.is_shutdown():
        pass

    # Calculate mean and standard deviation once done collecting data. Print to console.
    raw_statistics, filtered_statistics = aruco_pose_tracker.calculateMeanAndStd()
    rospy.loginfo("\nStatistics for raw pose data:\n\n" + str(raw_statistics))
    rospy.loginfo("\nStatistics for filtered data:\n\n" + str(filtered_statistics))
