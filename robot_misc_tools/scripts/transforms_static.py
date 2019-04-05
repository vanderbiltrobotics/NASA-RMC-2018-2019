#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from geometry_msgs.msg import Pose

# Import other required packages
import json
import sys

# Path to current directory
PATH_TO_DIR = sys.path[0] + "/tf_configs_static/"


# Converts a list of transforms stored in dictionaries to a list of transform messages
def get_transform_msgs(transforms):

    # List to store messages
    transform_msgs = []

    # Create a tf message from each tf in input list
    for transform in transforms:

        # Create new tf message
        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = rospy.Time.now()

        # Extract parent and child frame ids
        tf_msg.header.frame_id = transform["frame_id"]
        tf_msg.child_frame_id = transform["child_id"]

        # Extract translation
        tf_msg.transform.translation.x = transform["trans_x"]
        tf_msg.transform.translation.y = transform["trans_y"]
        tf_msg.transform.translation.z = transform["trans_z"]

        # Extract rotation
        tf_msg.transform.rotation.x = transform["rot_x"]
        tf_msg.transform.rotation.y = transform["rot_y"]
        tf_msg.transform.rotation.z = transform["rot_z"]
        tf_msg.transform.rotation.w = transform["rot_w"]

        # Add to the list
        transform_msgs.append(tf_msg)

    # Return the list of tf msgs
    return transform_msgs


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transforms_static")

    # Read in the name of transform file from parameter server
    file_name = rospy.get_param("tf_file_static", default="elevation_mapping_test.json")

    # Load file contents into a dictionary
    with open(PATH_TO_DIR + file_name) as f:
        transform_dict = json.load(f)

    # Extract list of transform objects
    transform_list = transform_dict["transforms"]

    # Create a broadcaster for publishing static transforms
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    # Convert the list to transform messages, broadcast them
    broadcaster.sendTransform(get_transform_msgs(transform_list))

    # Spin indefinitely
    rospy.spin()

