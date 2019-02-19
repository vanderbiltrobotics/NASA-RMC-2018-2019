#!/usr/bin/env python

# ----------------------------- #
# Subscribes to drive command, Publishes custom motor speeds. #
# ----------------------------- #

# Import ROS packages
import rospy
from visualization_msgs.msg import *


class RectangleMarker(Marker):

    def __init__(self, frame_id, pose_array, x_len, y_len, z_len, color_array):

        # Call init for super class
        super(RectangleMarker, self).__init__()

        # -- Fill message fields appropriately -- #

        # Basic info
        self.header.frame_id = frame_id
        self.type = Marker.CUBE

        # Pose the x, y, z dims are relative to
        self.pose.position.x = pose_array[0] + (x_len / 2.0)
        self.pose.position.y = pose_array[1] + (y_len / 2.0)
        self.pose.position.z = pose_array[2] + (z_len / 2.0)
        self.pose.orientation.x = pose_array[0]
        self.pose.orientation.y = pose_array[1]
        self.pose.orientation.z = pose_array[2]
        self.pose.orientation.w = pose_array[3]

        # Set x, y, z dims
        self.scale.x = x_len
        self.scale.y = y_len
        self.scale.z = z_len

        # Set colors
        self.color.r = color_array[0]
        self.color.g = color_array[1]
        self.color.b = color_array[2]
        self.color.a = color_array[3]




if __name__ == "__main__":

    # Initalize ROS node
    rospy.init_node("field_visualization")

    # Create publisher for markers
    marker_pub = rospy.Publisher("vizualization/field_markers", Marker, queue_size=0)

    # --- CREATE VISUALIZATION OBJECTS --- #

    # Ground
    ground_pose = [0, 0, 0, 0.707, 0, 0, 0.707]
    ground_color = [1.0, 0.0, 1.0, 1.0]
    ground_msg = RectangleMarker("world", ground_pose, 0.5, 0.2, 1.0, ground_color)

    # Front Wall

    # Back Wall

    # Left Wall

    # Right Wall

    # Loop rate
    loop_rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        # Publish ground
        marker_pub.publish(ground_msg)

        # Sleep based on loop rate
        loop_rate.sleep()




