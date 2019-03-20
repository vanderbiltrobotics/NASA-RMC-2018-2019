#!/usr/bin/env python

# Import ROS packages
import rospy
import tf2_ros
from geometry_msgs.msg import Pose

# Import other required packages
import json
import sys

# Path to current directory
PATH_TO_DIR = sys.path[0] + "/tf_configs_dynamic/"


class PoseToTF():

    # Constructor
    def __init__(self, pose_topic, child_id, frame_id):

        # Subscriber to pose topic
        self.pose_sub = rospy.Subscriber(pose_topic, Pose, self.tf_from_pose)

        # TF broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster()

        # Create base transform message
        self.tf_msg = tf2_ros.TransformStamped()
        self.tf_msg.header.frame_id = frame_id
        self.tf_msg.child_frame_id = child_id

    # Callback function for pose messages. Converts to tf msg and publishes
    def tf_from_pose(self, pose_msg):

        # Update message timestamp
        self.tf_msg.header.stamp = rospy.Time.now()

        # Update translation
        self.tf_msg.transform.translation.x = pose_msg.position.x
        self.tf_msg.transform.translation.y = pose_msg.position.y
        self.tf_msg.transform.translation.z = pose_msg.position.z

        # Update rotation
        self.tf_msg.transform.rotation.x = pose_msg.orientation.x
        self.tf_msg.transform.rotation.y = pose_msg.orientation.y
        self.tf_msg.transform.rotation.z = pose_msg.orientation.z
        self.tf_msg.transform.rotation.w = pose_msg.orientation.w

        # Publish transform
        self.broadcaster.sendTransform(self.tf_msg)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("transforms_dynamic")

    # Read in the name of transform file from parameter server
    file_name = rospy.get_param("tf_file_dynamic", default="full_system.json")

    # Load file contents into a dictionary
    with open(PATH_TO_DIR + file_name) as f:
        transform_dict = json.load(f)

    # Extract list of transform specifications
    transform_list = transform_dict["transforms"]

    # Create a PoseToTF object for each pose topic specified in file
    for transform in transform_list:

        # Extract data from dictionary
        frame_id = transform["frame_id"]
        child_id = transform["child_id"]
        pose_topic = transform["pose_topic"]

        # Create a new PoseToTF object
        PoseToTF(pose_topic, child_id, frame_id)

    # Spin indefinitely
    rospy.spin()

