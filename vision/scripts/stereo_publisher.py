

# Import ROS packages
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from dynamic_reconfigure.server import Server
from vision.cfg import DisparityConfig

# Other required packages
import cv2
import cv2.ximgproc as ximgproc
import numpy as np
import time
import matplotlib.pyplot as plt

# Class for storing reconfigurable disparity matcher parameters
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
        self.prefilter_cap = 31
        self.prefilter_size = 9
        self.prefilter_type = 1
        self.uniqueness_ratio = 15
        self.texture_threshold = 10
        self.smaller_block_size = 0

        # Other config settings
        self.use_wls_filter = False
        self.use_prefilter = False

    # Callback function for dynamic configure server
    def config_callback(self, config, level):

        # Update general params
        self.block_size = config["block_size"]
        self.min_disparity = config["min_disparity"]
        self.num_disparities = config["num_disparities"]
        self.disp12_max_diff = config["disp12_max_diff"]
        self.speckle_range = config["speckle_range"]
        self.speckle_window_size = config["speckle_window_size"]

        # Check if we want to update the pre-filtering parameters
        self.use_prefilter = config["use_prefilter"]

        if self.use_prefilter:
            self.prefilter_cap = config["prefilter_cap"]
            self.prefilter_size = config["prefilter_size"]
            self.prefilter_type = config["prefilter_type"]
            self.uniqueness_ratio = config["uniqueness_ratio"]
            self.texture_threshold = config["texture_threshold"]
            self.smaller_block_size = config["smaller_block_size"]

        return config

    # Updates the settings for the provided stereo matcher with the currently stored values
    def update_matcher(self, matcher):

        matcher.setBlockSize(self.block_size)
        matcher.setMinDisparity(self.min_disparity)
        matcher.setNumDisparities(self.num_disparities)
        matcher.setDisp12MaxDiff(self.disp12_max_diff)
        matcher.setSpeckleRange(self.speckle_range)
        matcher.setSpeckleWindowSize(self.speckle_window_size)
        matcher.setPreFilterCap(self.prefilter_cap)
        matcher.setPreFilterSize(self.prefilter_size)
        matcher.setPreFilterType(self.prefilter_type)
        matcher.setUniquenessRatio(self.uniqueness_ratio)
        matcher.setTextureThreshold(self.texture_threshold)
        matcher.setSmallerBlockSize(self.smaller_block_size)

        return matcher


# Configuration params
DEFAULT_DEVICE_ID_A = 1
DEFAULT_DEVICE_ID_B = 2
DEFAULT_MAP_FILE = 'stereo_maps.npz'
DEFAULT_PUB_RATE = 1

# Other params
CALIBRATION_FILE_DIR = "camera_calibration/calibration_data/"

# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('image_publisher')

    # Ready to go
    rospy.loginfo("Image publisher initialized...")

    # Initialize publishers for image topics
    a_rect_pub = rospy.Publisher("stereo/image_rect_a", Image, queue_size=1)
    b_rect_pub = rospy.Publisher("stereo/image_rect_b", Image, queue_size=1)
    a_rect_comp_pub = rospy.Publisher("stereo/image_compressed_a", CompressedImage, queue_size=1)
    b_rect_comp_pub = rospy.Publisher("stereo/image_compressed_b", CompressedImage, queue_size=1)
    disparity_pub = rospy.Publisher("stereo/disparity", Image, queue_size=1)

    # Read configuration parameters
    device_id_a = rospy.get_param("device_id_a", DEFAULT_DEVICE_ID_A)
    device_id_b = rospy.get_param("device_id_b", DEFAULT_DEVICE_ID_B)
    map_file = rospy.get_param("map_file", DEFAULT_MAP_FILE)
    publish_rate = rospy.get_param("publish_rate", DEFAULT_PUB_RATE)

    # Camera setup
    stream_a = cv2.VideoCapture(device_id_a)
    stream_b = cv2.VideoCapture(device_id_b)
    maps = np.load(CALIBRATION_FILE_DIR + map_file)
    bridge = CvBridge()

    # Create disparity matcher and config handler
    config = StereoBMConfig()
    matcher_a = cv2.StereoBM_create(config.num_disparities, config.block_size)

    print matcher_a.getPreFilterType()
    print matcher_a.getPreFilterSize()
    print matcher_a.getPreFilterCap()
    print matcher_a.getUniquenessRatio()
    print matcher_a.getTextureThreshold()
    print matcher_a.getSmallerBlockSize()

    # Server for dynamic config params
    srv = Server(DisparityConfig, config.config_callback)

    # Loop indefinitely
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Get next frame
        ret_a, frame_a = stream_a.read()
        ret_b, frame_b = stream_b.read()

        # Rectify raw image
        img_rect_a = cv2.remap(frame_a, maps['ax'], maps['ay'], cv2.INTER_LINEAR)
        img_rect_b = cv2.remap(frame_b, maps['bx'], maps['by'], cv2.INTER_LINEAR)

        # Convert to grayscale for computing disparity
        img_a_gray = cv2.cvtColor(img_rect_a, cv2.COLOR_BGR2GRAY).T
        img_b_gray = cv2.cvtColor(img_rect_b, cv2.COLOR_BGR2GRAY).T

        # Update the stereo matcher with the latest parameters
        config.update_matcher(matcher_a)

        # Compute disparity image
        disp_a = matcher_a.compute(img_b_gray, img_a_gray)

        # Scale appropriately
        disp_a = np.fmax(disp_a / 16, 0.0) / config.num_disparities

        # Display image
        cv2.imshow('Disparity A', disp_a.T)
        cv2.waitKey(1)

        # # Prepare messages
        # msg_rect_a = bridge.cv2_to_imgmsg(img_rect_a, encoding="bgr8")
        # msg_rect_b = bridge.cv2_to_imgmsg(img_rect_b, encoding="bgr8")
        # msg_rect_comp_a = bridge.cv2_to_compressed_imgmsg(img_rect_a)
        # msg_rect_comp_b = bridge.cv2_to_compressed_imgmsg(img_rect_b)
        # msg_disparity = bridge.cv2_to_imgmsg(disparity_img, encoding="bgr8")
        #
        # # Publish messages
        # a_rect_pub.publish(msg_rect_a)
        # b_rect_pub.publish(msg_rect_b)
        # b_rect_comp_pub.publish(msg_rect_comp_a)
        # b_rect_comp_pub.publish(msg_rect_comp_b)
        # disparity_pub.publish(msg_disparity)
        #
        # # Sleep
        rate.sleep()



# Set up filter
# right_matcher = ximgproc.createRightMatcher(left_matcher)
# visual_multiplier = 1.0
# filter = ximgproc.createDisparityWLSFilter(left_matcher)
# filter.setLambda(8000)
# filter.setSigmaColor(1.5)
# filtered_img = np.fmax(filtered_img / 16, 0.0) / num_disparities