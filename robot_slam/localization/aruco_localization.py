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
import tf2_ros

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
    "rot_w": SQRT_2 / 2.0,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": SQRT_2 / 2.0,   # 90-degree rotation in z
}
south_pose = Pose()  # gets initialized in __init__
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
bin_pose = Pose()
#  small aruco marker on the collection bin (3)
aruco_marker_smbin_pos_default = {
    "trans_x": 0.0, # TODO: Change.
    "trans_y": 0.5,
    "trans_z": 1.1, # TODO: Change.
    "rot_w": 1.0,
    "rot_x": 0.0,
    "rot_y": 0.0,
    "rot_z": 0.0,
}
smbin_pose = Pose()


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

        # tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Get params to create Aruco boards
        # south board
        south_markers_x = 1
        south_markers_y = 1
        south_marker_len = 0.168
        south_marker_sep = 0.0
        # bin board
        bin_markers_x = 2
        bin_markers_y = 1
        bin_marker_len = 0.168
        bin_marker_sep = 0.048
        # small bin board
        smbin_markers_x = 5
        smbin_markers_y = 1
        smbin_marker_len = 0.067
        smbin_marker_sep = 0.0135

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
        self.aruco_marker_smbin_pos = rospy.get_param(
            "aruco_small_marker_bin_pos", 
            default = aruco_marker_smbin_pos_default
        )

        # set the poses of the boards
        south_pose.position.x = self.aruco_marker_south_pos["trans_x"]
        south_pose.position.y = self.aruco_marker_south_pos["trans_y"]
        south_pose.position.z = self.aruco_marker_south_pos["trans_z"]
        south_pose.orientation.x = self.aruco_marker_south_pos["rot_x"]
        south_pose.orientation.y = self.aruco_marker_south_pos["rot_y"]
        south_pose.orientation.z = self.aruco_marker_south_pos["rot_z"]
        south_pose.orientation.w = self.aruco_marker_south_pos["rot_w"]
        bin_pose.position.x = self.aruco_marker_bin_pos["trans_x"]
        bin_pose.position.y = self.aruco_marker_bin_pos["trans_y"]
        bin_pose.position.z = self.aruco_marker_bin_pos["trans_z"]
        bin_pose.orientation.x = self.aruco_marker_bin_pos["rot_x"]
        bin_pose.orientation.y = self.aruco_marker_bin_pos["rot_y"]
        bin_pose.orientation.z = self.aruco_marker_bin_pos["rot_z"]
        bin_pose.orientation.w = self.aruco_marker_bin_pos["rot_w"]
        smbin_pose.position.x = self.aruco_marker_smbin_pos["trans_x"]
        smbin_pose.position.y = self.aruco_marker_smbin_pos["trans_y"]
        smbin_pose.position.z = self.aruco_marker_smbin_pos["trans_z"]
        smbin_pose.orientation.x = self.aruco_marker_smbin_pos["rot_x"]
        smbin_pose.orientation.y = self.aruco_marker_smbin_pos["rot_y"]
        smbin_pose.orientation.z = self.aruco_marker_smbin_pos["rot_z"]
        smbin_pose.orientation.w = self.aruco_marker_smbin_pos["rot_w"]

        # Create aruco boards
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.aruco_param = aruco.DetectorParameters_create()  # default parameters
        self.south_board = aruco.GridBoard_create(
            south_markers_x, 
            south_markers_y, 
            south_marker_len, 
            south_marker_sep, 
            self.aruco_dict
        )
        self.bin_board = aruco.GridBoard_create(
            bin_markers_x, 
            bin_markers_y, 
            bin_marker_len, 
            bin_marker_sep, 
            self.aruco_dict
        )
        self.smbin_board = aruco.GridBoard_create(
            smbin_markers_x, 
            smbin_markers_y, 
            smbin_marker_len, 
            smbin_marker_sep, 
            self.aruco_dict
        )

        # markerNumber is 1 for the x-aruco, 2 for the big y-aruco
        # and 3 for the small y-aruco
        self.marker_number = 1
        # the aruco board will change depending on the marker number
        self.active_board = self.south_board

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
        aruco.refineDetectedMarkers(cv_image, self.active_board, corners, ids, rejectedImgPoints, self.cmatx, self.dist)
        # rospy.loginfo(corners) # test
        # print(np.shape(corners))

        # If markers found, estimate pose
        if ids is not None:
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.active_board, self.cmatx, self.dist)
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

        robot_pose = self.tf_buffer.lookup_transform(
            self.world_frame_id, 
            self.robot_frame_id, 
            rospy.Time()
        )
        x = robot_pose.transform.translation.x
        y = robot_pose.transform.translation.y
        pose_msg = Pose()

        # switch depending on what aruco marker we're currently looking at
        # TODO: this needs to be tested/debugged
        if self.marker_number == 1:
            if 0.8 * x + 2 > y and x < 1.9:
                self.marker_number = 3  # small bin board takes priority
                self.active_board = self.smbin_board
            elif x > y:
                self.marker_number = 2
                self.active_board = self.bin_board

        elif self.marker_number == 2:
            if 0.8 * x + 2 > y and x < 1.9:
                self.marker_number = 3  # small bin board takes priority
                self.active_board = self.smbin_board
            elif x + 2 < y:
                self.marker_number = 1
                self.active_board = self.south_board

        elif self.marker_number == 3:
            if x > y and x < 2.5:
                self.marker_number = 2  # big bin board takes priority
                self.active_board = self.bin_board
            elif x + 2 < y:
                self.marker_number = 1
                self.active_board = self.south_board

        # publish the pose of the active board
        if self.marker_number == 1:
            self.aruco_pos_pub.publish(south_pose)
        elif self.marker_number == 2:
            self.aruco_pos_pub.publish(bin_pose)
        elif self.marker_number == 3:
            self.aruco_pos_pub.publish(smbin_pose)


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
