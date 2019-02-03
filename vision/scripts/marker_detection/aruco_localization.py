#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image

def image_callback(data):



if __name__ == "__main__":
    rospy.init_node('aruco_localization')
    image_sub = rospy.Subscriber('aruco_image', Image, image_callback)

#initialize node
#create subscriber to image frames
#create subscribers for transforms
#define callback function that runs when image is received
