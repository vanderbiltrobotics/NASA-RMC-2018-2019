#!/usr/bin/env python

# Import ROS packages
import rospy
import rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import tf.transformations

# Import other required packages
import cv2.aruco as aruco
import cv2
import numpy as np
import yaml


DEFAULT_CALIB_FILE = 'camera_a.yaml'
CALIBRATION_FILE_DIR = rospkg.RosPack().get_path("robot_sensors") + '/cameras/camera_calibration/calibration_data/'

class ImageHandler:

    # Constructor
    def __init__(self):

        # Load calibration file
        cal_data = yaml.load(open(CALIBRATION_FILE_DIR + DEFAULT_CALIB_FILE, 'r'))
        self.cmatx = np.asarray(cal_data["camera_matrix"])
        self.dist = np.asarray(cal_data["dist_coefficients"])

        ## GET RID OF ASSUMING LAUNCH FILE PARAMS WORK
        # markerLength = .029  # m; determine later
        # markerSeparation = .006  # m; determine later
        ##

        # Get params to create Aruco board
        self.numMarkersX = rospy.get_param("board_x_markers", 0)
        self.numMarkersY = rospy.get_param("board_y_markers", 0)
        self.markerLength = rospy.get_param("board_marker_length", 0)
        self.markerSeparation = rospy.get_param("board_marker_separation", 0)

        # Set up aruco boards
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # default parameters
        self.aruco_board = aruco.GridBoard_create(self.numMarkersX, self.numMarkersY, self.markerLength, self.markerSeparation, self.aruco_dict)

        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher("aruco/pose_raw", Pose, queue_size=0)
        self.bool_pub = rospy.Publisher("aruco/marker_detected", Bool, queue_size=0)
        self.avg_of_corners = rospy.Publisher("aruco/avg_of_corners", Int32, queue_size=0)
        self.bridge = CvBridge()

    # Callback for received image frames
    def image_callback(self, data):

        # Define variables for messages to publish
        pose_msg = Pose()
        bool_msg = Bool()
        avg_corners = 0

        # Convert from ROS message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Find corners and IDs of aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_param)
        aruco.refineDetectedMarkers(cv_image, self.aruco_board, corners, ids, rejectedImgPoints, self.cmatx, self.dist)
        # rospy.loginfo(corners) # test
        # print(np.shape(corners))

        # If markers found, estimate pose
        if ids is not None:
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.aruco_board, self.cmatx, self.dist)
            bool_msg.data = retval


            # If succesful, convert rvec from rpy to quaternion, fill pose message
            if retval:

                quat = tf.transformations.quaternion_from_euler(rvec[1], rvec[0], rvec[2])

                # Store pose and quaternion information of Aruco Board
                pose_msg.position.x = tvec[2]
                pose_msg.position.y = -tvec[0]
                pose_msg.position.z = tvec[1]
                pose_msg.orientation.x = quat[0]
                pose_msg.orientation.y = quat[1]
                pose_msg.orientation.z = quat[2]
                pose_msg.orientation.w = quat[3]

                # Calculate the average x distance between the corners 
                # avg_corners = np.min(corners[:, 0, :, 0])
                # print(type(corners[0]))
                bla = corners[0][0][:, 0]
                min = corners[0][0, 0, 0]
                max = corners[0][0, 0, 0]

                for corner in corners:
                    if np.min(corner[0, :, 0]) < min:
                        min = np.min(corner[0, :, 0])
                    elif np.max(corner[0, :, 0]) > max:
                        max = np.max(corner[0, :, 0])

                self.avg_of_corners.publish((min + max)/2.0)

        else:
            bool_msg.data = False
            self.avg_of_corners.publish(avg_corners)

        # Publish messages
        self.pose_pub.publish(pose_msg)
        self.bool_pub.publish(bool_msg)


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
