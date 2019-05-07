#!/usr/bin/env python

# This program publishes the location of the aruco marker, determines
# which of the three field aruco markers the robot localizes to, and
# sweeps the servo when the aruco marker is missing for an extended 
# period of time.

# Created by Alex Barnett, Jacob Gloudemans, and Josh Petrin

# Import ROS packages
import rospy
import rospkg
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import tf.transformations
import tf2_ros

# Import other required packages
import cv2.aruco as aruco
import cv2
import numpy as np
import yaml

import math

# Camera parameters
DEFAULT_CALIB_FILE = 'camera_a.yaml'
CALIBRATION_FILE_DIR = rospkg.RosPack().get_path("robot_sensors") + '/cameras/camera_calibration/calibration_data/'

# --- POSES OF EACH MARKER BOARD RELATIVE TO WORLD --- #

# Pose of south 2x1 marker board
south_2x1_pose = Pose()
south_2x1_pose.position.x = 0.92
south_2x1_pose.position.y = 0.27
south_2x1_pose.position.z = 0.295
south_2x1_pose.orientation.x = 0.0
south_2x1_pose.orientation.y = 0.0
south_2x1_pose.orientation.z = 0.707
south_2x1_pose.orientation.w = 0.707

# Pose of west 2x1 marker board
west_2x1_pose = Pose()
west_2x1_pose.position.x = 0.46
west_2x1_pose.position.y = 0.33
west_2x1_pose.position.z = 0.295
west_2x1_pose.orientation.x = 0.0
west_2x1_pose.orientation.y = 0.0
west_2x1_pose.orientation.z = 0.0
west_2x1_pose.orientation.w = 1.0

# Pose of west 4x3 marker board
west_4x3_pose = Pose()
west_4x3_pose.position.x = 0.46
west_4x3_pose.position.y = 0.667
west_4x3_pose.position.z = 0.311
west_4x3_pose.orientation.x = 0.0
west_4x3_pose.orientation.y = 0.0
west_4x3_pose.orientation.z = 0.0
west_4x3_pose.orientation.w = 1.0

# --- OTHER PARAMETERS FOR EACH BOARD --- #

# South 2x1 board params
south_2x1_markers_x = 2
south_2x1_markers_y = 1
south_2x1_marker_len = 0.177
south_2x1_marker_sep = 0.02
south_2x1_marker_idstart = 2

# West 2x1 board params
west_2x1_markers_x = 2
west_2x1_markers_y = 1
west_2x1_marker_len = 0.177
west_2x1_marker_sep = 0.02
west_2x1_marker_idstart = 0

# West 4x3 board params
west_4x3_markers_x = 4
west_4x3_markers_y = 3
west_4x3_marker_len = 0.05
west_4x3_marker_sep = 0.06
west_4x3_marker_idstart = 4

# --- CREATE THE ARUCO BOARDS --- #

# Stuff for all of the boards
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_1000)
ARUCO_PARAM = aruco.DetectorParameters_create()

# Create south 2x1 board
SOUTH_2x1_BOARD = aruco.GridBoard_create(
    south_2x1_markers_x,
    south_2x1_markers_y,
    south_2x1_marker_len,
    south_2x1_marker_sep,
    ARUCO_DICT,
    firstMarker=south_2x1_marker_idstart
)

# Create west 2x1 board
WEST_2x1_BOARD = aruco.GridBoard_create(
    west_2x1_markers_x,
    west_2x1_markers_y,
    west_2x1_marker_len,
    west_2x1_marker_sep,
    ARUCO_DICT,
    firstMarker=west_2x1_marker_idstart
)

# Create west 2x1 board
WEST_4x3_BOARD = aruco.GridBoard_create(
    west_4x3_markers_x,
    west_4x3_markers_y,
    west_4x3_marker_len,
    west_4x3_marker_sep,
    ARUCO_DICT,
    firstMarker=west_4x3_marker_idstart
)

class ImageHandler:

    # Constructor
    def __init__(self):

        # Load calibration file
        cal_data = yaml.load(open(CALIBRATION_FILE_DIR + DEFAULT_CALIB_FILE, 'r'))
        self.cmatx = np.asarray(cal_data["camera_matrix"])
        self.dist = np.asarray(cal_data["dist_coefficients"])

        # TF listener - use to get poses to update active board
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # tf frame id's
        self.world_frame_id = rospy.get_param("pp_world_frame_id", default="world")
        self.robot_frame_id = rospy.get_param("pp_robot_frame_id", default="robot_center")
        self.aruco_frame_id = rospy.get_param("pp_aruco_frame_id", default="aruco_board_origin")

        # Set to true when we first detect a marker
        self.first_marker_detected = False

        # ArUco board we're currently searching for
        self.active_board = SOUTH_2x1_BOARD
        self.active_board_pose = south_2x1_pose

        # Subscriber to images
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher("aruco/pose_raw", Pose, queue_size=0)
        self.bool_pub = rospy.Publisher("aruco/marker_detected", Bool, queue_size=0)
        self.avg_of_corners = rospy.Publisher("aruco/avg_of_corners", Int32, queue_size=0)
        self.aruco_pos_pub = rospy.Publisher("aruco/active_board_pose", Pose, queue_size=0)

        # CV bridge object for converting ROS images to opencv
        self.bridge = CvBridge()

        self.prev_rvec = None
        self.prev_tvec = None


    # Callback for received image frames
    def image_callback(self, data):

        # --- SETUP STUFF --- #

        # Update the board we're looking for
        self.update_aruco_target()

        # Define variables for messages to publish
        pose_msg = Pose()
        bool_msg = Bool()
        avg_corners = 0

        # --- IMAGE PROCESSING --- #

        # Convert from ROS message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

        # Find corners and IDs of aruco markers
        corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, ARUCO_DICT, parameters=ARUCO_PARAM)

        # Refine for improved accuracy
        aruco.refineDetectedMarkers(cv_image, SOUTH_2x1_BOARD, corners, ids, rejectedImgPoints, self.cmatx, self.dist)

        # If markers found, estimate pose
        if ids is not None:

            aruco.drawDetectedMarkers(cv_image, corners, ids)

            if self.prev_rvec is None:
                _, self.prev_rvec, self.prev_tvec = aruco.estimatePoseBoard(corners, ids, SOUTH_2x1_BOARD, self.cmatx, self.dist)

            # Compute pose estimate based on board corner positions
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, SOUTH_2x1_BOARD, self.cmatx, self.dist, rvec=self.prev_rvec, tvec=self.prev_tvec, useExtrinsicGuess=True)

            # If succesful, convert rvec from rpy to quaternion, fill pose message
            if retval != 0:

                # Set param to true
                self.first_marker_detected = True

                self.prev_tvec = tvec
                self.prev_rvec = rvec

                # - IMPORTANT - "quaternion_from_euler" is STUPID and changes the values of rvec - if you want to
                # use rvec after calling this function, you need to make a copy and use the copy
                copy = np.copy(rvec)

                # Convert the 'rvec' from rpy to a quaternion
                quat = tf.transformations.quaternion_from_euler(rvec[1], rvec[0], rvec[2])

                # Store pose and quaternion information of Aruco Board
                pose_msg.position.x = tvec[2]
                pose_msg.position.y = tvec[0]
                pose_msg.position.z = tvec[1]
                pose_msg.orientation.x = 0.0 #quat[0]
                pose_msg.orientation.y = 0.0 #quat[1]
                pose_msg.orientation.z = 0.0 #quat[2]
                pose_msg.orientation.w = 1.0 #quat[3]

                # Calculate the average x distance between the corners
                min = corners[0][0, 0, 0]
                max = corners[0][0, 0, 0]

                for corner in corners:
                    if np.min(corner[0, :, 0]) < min:
                        min = np.min(corner[0, :, 0])
                    elif np.max(corner[0, :, 0]) > max:
                        max = np.max(corner[0, :, 0])

                # Publish pose estimate
                self.pose_pub.publish(pose_msg)
                self.avg_of_corners.publish((min + max) / 2.0)

            # Update boolean message
            bool_msg.data = retval

        else:
            bool_msg.data = False

        cv2.imshow("image", cv_image)
        cv2.waitKey(1)

        # Always publish whether markers were found or not
        self.avg_of_corners.publish(avg_corners)
        self.bool_pub.publish(bool_msg)

    # Update which aruco marker we're looking for. Run every ~1 s.
    # Senses the robot's position and
    # 1) sets the target to the south marker if the position is far down
    # 2) sets to the bin marker if the robot is upfield
    # 3) sets to the small bin marker if the robot is close to the bin
    def update_aruco_target(self):

        # Check if we can get robot pose from tf tree
        if self.first_marker_detected:

            # Get the robot's x, y position from the tf tree
            robot_pose = self.tf_buffer.lookup_transform(self.world_frame_id, self.robot_frame_id, rospy.Time())
            x = robot_pose.transform.translation.x
            y = robot_pose.transform.translation.y

            if self.active_board == SOUTH_2x1_BOARD:

                if 0.8 * x + 2 > y and x < 1.9:
                    self.active_board = WEST_4x3_BOARD
                    self.active_board_pose = west_4x3_pose

                elif x > y:
                    self.active_board = WEST_2x1_BOARD
                    self.active_board_pose = west_2x1_pose

            elif self.active_board == WEST_2x1_BOARD:

                if 0.8 * x + 2 > y and x < 1.9:
                    self.active_board = WEST_4x3_BOARD
                    self.active_board_pose = west_4x3_pose

                elif x + 2 < y:
                    self.active_board = SOUTH_2x1_BOARD
                    self.active_board_pose = south_2x1_pose

            elif self.active_board == WEST_4x3_BOARD:

                if x > y and x > 2.5:
                    self.active_board = WEST_2x1_BOARD
                    self.active_board_pose = west_2x1_pose

                elif x + 2 < y:
                    self.active_board = WEST_4x3_BOARD
                    self.active_board_pose = west_4x3_pose

        # Default to WEST 2x1 marker if transform doesn't exist yet
        else:
            self.active_board = SOUTH_2x1_BOARD
            self.active_board_pose = south_2x1_pose

        # Publish the pose of the active board
        self.active_board = SOUTH_2x1_BOARD
        self.aruco_pos_pub.publish(south_2x1_pose)


if __name__ == "__main__":

    # Initialize as ROS node
    rospy.init_node('aruco_localization')

    # Create subscriber for aruco image frames
    handler = ImageHandler()

    rospy.spin()
