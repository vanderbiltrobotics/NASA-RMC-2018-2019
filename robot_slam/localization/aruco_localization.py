#!/usr/bin/env python

# This program publishes the location of the aruco marker, determines
# which of the three field aruco markers the robot localizes to, and
# sweeps the servo when the aruco marker is missing for an extended 
# period of time.

# Created by Jacob Gloudemans and Josh Petrin


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
import math
import numpy as np
import yaml

# constant(s)
SQRT_2 = math.sqrt(2)

# default locations of the aruco markers -- 
#  aruco marker on the short wall of the arena (1)
aruco_marker_south_pos_default = {
    "trans_x": 1.5, # TODO: Change.
    "trans_y": 0.0,
    "trans_z": 1.0, # TODO: Change.
    "rot_w": SQRT_2 / 2,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": SQRT_2 / 2, # 90-degree rotation in z
}
#  aruco marker on the collection bin (2)
aruco_marker_bin_pos_default = {
    "trans_x": 0.0, # TODO: Change.
    "trans_y": 0.6,
    "trans_z": 1.0, # TODO: Change.
    "rot_w": 1.0,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": 0.0,
}
#  small aruco marker on the collection bin (3)
aruco_small_marker_bin_pos_default = {
    "trans_x": 0.0, # TODO: Change.
    "trans_y": 0.5,
    "trans_z": 1.1, # TODO: Change.
    "rot_w": 1.0,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": 0.0,
}


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

        # tf listeners
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # markerNumber is 1 for the x-aruco, 2 for the big y-aruco
        # and 3 for the small y-aruco
        self.markerNumber = 1

        # Get params to create Aruco board
        self.numMarkersX = rospy.get_param("board_x_markers", 0)
        self.numMarkersY = rospy.get_param("board_y_markers", 0)
        self.markerLength = rospy.get_param("board_marker_length", 0)
        self.markerSeparation = rospy.get_param("board_marker_separation", 0)
        self.world_frame_id = rospy.get_param("pp_world_frame_id", default="world")
        self.robot_frame_id = rospy.get_param("pp_robot_frame_id", default="robot_center")
        self.aruco_frame_id = rospy.get_param("pp_aruco_frame_id", default="aruco_board_origin")
        # tf frame id's
        self.world_frame_id = rospy.get_param("pp_world_frame_id", default="world")
        self.robot_frame_id = rospy.get_param("pp_robot_frame_id", default="robot_center")
        self.aruco_frame_id = rospy.get_param("pp_aruco_frame_id", default="aruco_board_origin")
        # params for the posititions of the aruco markers
        # aruco marker on the short side of the field (1)
        self.aruco_marker_south_pos = rospy.get_param(
            "aruco_marker_south_pos", 
            default = aruco_marker_south_pos_default
        )
        # aruco marker on the collecting bin (2)
        self.aruco_marker_bin_pos = rospy.get_param(
            "aruco_marker_bin_pos", 
            default = aruco_marker_bin_pos_default
        )
        # small aruco marker on the collecting bin (3)
        self.aruco_marker_bin_pos = rospy.get_param(
            "aruco_small_marker_bin_pos", 
            default = aruco_small_marker_bin_pos_default
        )

        # Set up aruco boards
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # default parameters
        self.aruco_board = aruco.GridBoard_create(self.numMarkersX, self.numMarkersY, self.markerLength, self.markerSeparation, self.aruco_dict)

        # Subscriber to images
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        
        # Publishers
        self.pose_pub = rospy.Publisher("aruco/pose_raw", Pose, queue_size=0)
        self.bool_pub = rospy.Publisher("aruco/marker_detected", Bool, queue_size=0)
        self.avg_of_corners = rospy.Publisher("aruco/avg_of_corners", Int32, queue_size=0)
        self.aruco_pos_pub = rospy.Publisher("aruco/active_board_position", Pose, queue_size=0)
        
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


    # Update which aruco marker we're looking for. Run every ~1 s.
    # Senses the robot's position and
    # 1) sets the target to the south marker if the position is far down
    # 2) sets to the bin marker if the robot is upfield
    # 3) sets to the small bin marker if the robot is close to the bin
    def update_aruco_target(self):



if __name__ == "__main__":

    # Initialize as ROS node
    rospy.init_node('aruco_localization')

    # Create subscriber for aruco image frames
    handler = ImageHandler()

    # update the aruco target every once in a while
    loop_rate = rospy.Rate(1)
    # Run indefinitely
    while not rospy.is_shutdown():
        handler.update_aruco_target()
        loop_rate.sleep()

