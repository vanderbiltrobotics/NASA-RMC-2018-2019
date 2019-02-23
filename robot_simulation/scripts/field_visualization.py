#!/usr/bin/env python

# ----------------------------- #
# Subscribes to drive command, Publishes custom motor speeds. #
# ----------------------------- #

# Import ROS packages
import rospy
from visualization_msgs.msg import *


class RectangleMarker(Marker):

    # Increment when adding markers to give them new id's
    count = 0

    def __init__(self, frame_id, pose_array, x_len, y_len, z_len, color_array):

        # Call init for super class
        super(RectangleMarker, self).__init__()

        # -- Fill message fields appropriately -- #

        # Basic info
        self.header.frame_id = frame_id
        self.id = RectangleMarker.count
        self.type = Marker.CUBE

        # Update count
        RectangleMarker.count += 1

        # Pose the x, y, z dims are relative to
        self.pose.position.x = pose_array[0] + (x_len / 2.0)
        self.pose.position.y = pose_array[1] + (y_len / 2.0)
        self.pose.position.z = pose_array[2] + (z_len / 2.0)
        self.pose.orientation.x = pose_array[3]
        self.pose.orientation.y = pose_array[4]
        self.pose.orientation.z = pose_array[5]
        self.pose.orientation.w = pose_array[6]

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
    marker_pub = rospy.Publisher("vizualization/arena_markers", MarkerArray, queue_size=0)
    marker_array = MarkerArray()

    # --- CREATE VISUALIZATION OBJECTS --- #

    # Dimensions
    wall_height = 0.3
    wall_width = 0.1
    field_length = 7.31
    field_width = 3.81

    # Colors
    wall_color = [0.25, 0.25, 0.25, 1.0]
    ground_color = [0.9, 0.8, 0.7, 1.0]

    # -- Add walls -- #

    # Front
    wall_front_pose = [-wall_width, -wall_width, 0, 0, 0, 0, 0]
    wall_front_msg = RectangleMarker("world", wall_front_pose, field_width + (wall_width * 2),
                                     wall_width, wall_height, wall_color)

    # Back
    wall_back_pose = [-wall_width, field_length, 0, 0, 0, 0, 0]
    wall_back_msg = RectangleMarker("world", wall_back_pose, field_width + (wall_width * 2),
                                    wall_width, wall_height, wall_color)

    # Left
    wall_left_pose = [-wall_width, 0, 0, 0, 0, 0, 0]
    wall_left_msg = RectangleMarker("world", wall_left_pose, wall_width,
                                    field_length, wall_height, wall_color)

    # Right
    wall_right_pose = [field_width, 0, 0, 0, 0, 0, 0]
    wall_right_msg = RectangleMarker("world", wall_right_pose, wall_width,
                                     field_length, wall_height, wall_color)

    # -- Add ground -- #

    ground_pose = [-wall_width, -wall_width, 0, 0, 0, 0, 0]
    ground_msg = RectangleMarker("world", ground_pose, field_width + (wall_width * 2), field_length + (wall_width * 2), 0.001, ground_color)

    # Add all the messages to the marker array
    marker_array.markers.append(wall_front_msg)
    marker_array.markers.append(wall_back_msg)
    marker_array.markers.append(wall_left_msg)
    marker_array.markers.append(wall_right_msg)
    marker_array.markers.append(ground_msg)

    # Loop rate
    loop_rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        # Publish walls
        marker_pub.publish(marker_array)

        # Sleep based on loop rate
        loop_rate.sleep()




