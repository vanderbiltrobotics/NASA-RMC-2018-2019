#!/usr/bin/env python

# Import ROS packages
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import tf.transformations

# Import other required packages
import cv2.aruco as aruco
import numpy as np
import yaml
import sys


DEFAULT_CALIB_FILE = 'camera_a.yaml'
CALIBRATION_FILE_DIR = sys.path[0] + '/../camera_calibration/calibration_data/'


class ImageHandler:

    # Constructor
    def __init__(self):

        # Load calibration file
        cal_data = yaml.load(open(CALIBRATION_FILE_DIR + DEFAULT_CALIB_FILE, 'r'))
        self.cmatx = np.asarray(cal_data["camera_matrix"])
        self.dist = np.asarray(cal_data["dist_coefficients"])

        # Set up aruco boards
        markerLength = .029  # m; determine later
        markerSeparation = .006  # m; determine later
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # default parameters
        self.aruco_board = aruco.GridBoard_create(5, 7, markerLength, markerSeparation, self.aruco_dict)

        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher("aruco/pose_raw", Pose, queue_size=0)
        self.bool_pub = rospy.Publisher("aruco/marker_detected", Bool, queue_size=0)
        self.bridge = CvBridge()

    # Callback for received image frames
    def image_callback(self, data):

        # Define variables for messages to publish
        pose_msg = Pose()
        bool_msg = Bool()

        # Convert from ROS message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Find corners and IDs of aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_param)
        aruco.refineDetectedMarkers(cv_image, self.aruco_board, corners, ids, rejectedImgPoints, self.cmatx, self.dist)

        # If markers found, estimate pose
        if ids is not None:
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.aruco_board, self.cmatx, self.dist)
            bool_msg.data = retval

            # If succesful, convert rvec fromm rpy to quaternion, fill pose message
            if retval:

                quat = tf.transformations.quaternion_from_euler(rvec[0], rvec[1], rvec[2])

                pose_msg.position.x = tvec[0]
                pose_msg.position.y = tvec[1]
                pose_msg.position.z = tvec[2]
                pose_msg.orientation.x = quat[0]
                pose_msg.orientation.y = quat[1]
                pose_msg.orientation.z = quat[2]
                pose_msg.orientation.w = quat[3]

        else:
            bool_msg.data = False

        # Publish messages
        self.pose_pub.publish(pose_msg)
        self.bool_pub.publish(bool_msg)



        # Apply transforms to get coordinates in world_frame


if __name__ == "__main__":

    # Initialize as ROS node
    rospy.init_node('aruco_localization')

    # Create subscriber for aruco image frames
    handler = ImageHandler()

    # Run indefinitely
    rospy.spin()

#initialize node
#create subscriber to image frames
#create subscribers for transforms
#define callback function that runs when image is received
