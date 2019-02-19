#!/usr/bin/env python

# Import ROS packages
import rospy
from dynamic_reconfigure.server import Server
from vision.cfg import DisparityConfig

# Other required packages
import cv2
import cv2.ximgproc as ximgproc
import numpy as np
import matplotlib.pyplot as plt


# Path to test images
PATH_A = "ref_images/stereo_a/"
PATH_B = "ref_images/stereo_b/"

# Left and right images
img_a = cv2.imread(PATH_A + "calib_img2.png", cv2.IMREAD_GRAYSCALE)
img_b = cv2.imread(PATH_B + "calib_img2.png", cv2.IMREAD_GRAYSCALE)

# Image rectification stuff
DEFAULT_MAP_FILE = 'stereo_maps.npz'
CALIBRATION_FILE_DIR = "../camera_calibration/calibration_data/"
maps = np.load(CALIBRATION_FILE_DIR + DEFAULT_MAP_FILE)
img_rect_a = cv2.remap(img_a, maps['ax'], maps['ay'], cv2.INTER_LINEAR)
img_rect_b = cv2.remap(img_b, maps['bx'], maps['by'], cv2.INTER_LINEAR)


class StereoBMConfig:

    def __init__(self):

        # General stereo matcher params
        self.block_size = 7
        self.min_disparity = 1
        self.num_disparities = 112
        self.disp12_max_diff = 1
        self.speckle_range = 8
        self.speckle_window_size = 0

        # Stereo BM params
        self.prefilter_cap = 61
        self.prefilter_size = 5
        self.prefilter_type = 0
        self.uniqueness_ratio = 5
        self.texture_threshold = 501
        self.smaller_block_size = 5

    # Callback function for dynamic configure server
    def config_callback(self, config, level):

        print "Received parameter update"

        self.block_size = config["block_size"]
        self.min_disparity = config["min_disparity"]
        self.num_disparities = config["num_disparities"]
        self.disp12_max_diff = config["disp12_max_diff"]
        self.speckle_range = config["speckle_range"]
        self.speckle_window_size = config["speckle_window_size"]
        self.prefilter_cap = config["prefilter_cap"]
        self.prefilter_size = config["prefilter_size"]
        self.prefilter_type = config["prefilter_type"]
        self.uniqueness_ratio = config["uniqueness_ratio"]
        self.texture_threshold = config["texture_threshold"]
        self.smaller_block_size = config["smaller_block_size"]

        return config


# Create disparity match




# Launch ROS node
if __name__ == "__main__":

    # Initialize node
    rospy.init_node("disparity_tuning")

    # Create disparity matcher and config handler
    config = StereoBMConfig()
    left_matcher = cv2.StereoBM_create(config.num_disparities, config.block_size)

    # Server for dynamic config params
    srv = Server(DisparityConfig, config.config_callback)

    # Loop rate
    loop_rate = rospy.Rate(15)

    # Loop indefinitely
    while not rospy.is_shutdown():

        # Update parameters for disparity matcher
        left_matcher.setBlockSize(config.block_size)
        left_matcher.setMinDisparity(config.min_disparity)
        left_matcher.setNumDisparities(config.num_disparities)
        left_matcher.setDisp12MaxDiff(config.disp12_max_diff)
        left_matcher.setSpeckleRange(config.speckle_range)
        left_matcher.setSpeckleWindowSize(config.speckle_window_size)
        left_matcher.setPreFilterCap(config.prefilter_cap)
        left_matcher.setPreFilterSize(config.prefilter_size)
        left_matcher.setPreFilterType(config.prefilter_type)
        left_matcher.setUniquenessRatio(config.uniqueness_ratio)
        left_matcher.setTextureThreshold(config.texture_threshold)
        left_matcher.setSmallerBlockSize(config.smaller_block_size)

        # Compute new disparity image
        disp = left_matcher.compute(img_rect_b.T, img_rect_a.T)

        # Scale properly
        disp = np.fmax(disp / 16, 0.0) / config.num_disparities

        # Display image
        cv2.imshow("Dispartiy image", disp.T)
        cv2.imshow("Image A", img_rect_a)
        cv2.imshow("Image B", img_rect_b)
        cv2.waitKey(1)

