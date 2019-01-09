

# Import ROS packages
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# Other required packages
import cv2
import yaml
import numpy as np

# Configuration params
DEFAULT_DEVICE_ID = 0
DEFAULT_CALIB_FILE = 'camera_a.yaml'
DEFAULT_PUB_RATE = 1
DEFAULT_IMG_WIDTH = 640
DEFAULT_IMG_HEIGHT = 480
DEFAULT_ALPHA = 1

# Other params
CALIBRATION_FILE_DIR = "camera_calibration/calibration_data/"

# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('image_publisher')

    # Ready to go
    rospy.loginfo("Image publisher initialized...")

    # Initialize publishers for image topics
    img_raw_pub = rospy.Publisher("image_raw", Image, queue_size=1)
    img_comp_pub = rospy.Publisher("image_compressed", CompressedImage, queue_size=1)
    img_rect_pub = rospy.Publisher("image_rect", Image, queue_size=1)
    img_rect_comp_pub = rospy.Publisher("image_rect_compressed", CompressedImage, queue_size=1)

    # Read configuration parameters
    device_id = rospy.get_param("device_id", DEFAULT_DEVICE_ID)
    calibration_file = rospy.get_param("calibration_file", DEFAULT_CALIB_FILE)
    publish_rate = rospy.get_param("publish_rate", DEFAULT_PUB_RATE)
    img_height = rospy.get_param("image_height", DEFAULT_IMG_HEIGHT)
    img_width = rospy.get_param("image_width", DEFAULT_IMG_WIDTH)

    # Camera setup
    stream = cv2.VideoCapture(device_id)
    cal_data = yaml.load(open(CALIBRATION_FILE_DIR + calibration_file, 'r'))
    cmatx = np.asarray(cal_data["camera_matrix"])
    dist = np.asarray(cal_data["dist_coefficients"])
    new_cmatx, roi = cv2.getOptimalNewCameraMatrix(cmatx, dist, (img_width, img_height), alpha=DEFAULT_ALPHA)
    mapx, mapy = cv2.initUndistortRectifyMap(cmatx, dist, None, new_cmatx, (img_width, img_height), cv2.CV_32FC1)
    bridge = CvBridge()

    # Loop indefinitely
    rate = rospy.Rate(29)
    while not rospy.is_shutdown():

        # Get next frame
        ret, frame = stream.read()

        # Prepare raw, compressed image messages for un-rectified image
        msg_raw = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg_comp = bridge.cv2_to_compressed_imgmsg(frame)

        # Rectify raw image
        img_rect = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        # Prepare raw, compressed image messages for rectified image
        msg_rect = bridge.cv2_to_imgmsg(img_rect, encoding="bgr8")
        msg_rect_comp = bridge.cv2_to_compressed_imgmsg(img_rect)

        # Publish messages
        img_raw_pub.publish(msg_raw)
        img_comp_pub.publish(msg_comp)
        img_rect_pub.publish(msg_rect)
        img_rect_comp_pub.publish(msg_rect_comp)

        # Sleep
        rate.sleep()

