#!/usr/bin/env python
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import OccupancyGrid
import cv2
from math import degrees


class Field:

    def __init__(self, length, width, local_map_frame_id, global_map_frame_id):

        # Global map we're maintaing
        self.map = np.zeros(shape=(length, width))

        # Transform listener for getting latest pose information
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Frame IDs
        self.local_map_frame_id = local_map_frame_id
        self.global_map_frame_id = global_map_frame_id

    # Callback for new local map
    def update_field(self, new_map_msg):

        # Convert occupancy grid to numpy array
        local_map = np.reshape(new_map_msg.data, (new_map_msg.info.height, new_map_msg.info.width)).astype(np.float32)

        # Get current pose from transform listener
        cur_tf = self.tfBuffer.lookup_transform(self.global_map_frame_id, self.local_map_frame_id, rospy.Time(0))
        q = (cur_tf.transform.rotation.x,
             cur_tf.transform.rotation.y,
             cur_tf.transform.rotation.z,
             cur_tf.transform.rotation.w)  # quaternion from tf
        rpy = euler_from_quaternion(q)  # need yaw for the angle of the camera
        rpy = [degrees(rpy[0]), degrees(rpy[1]), degrees(rpy[2])]

        # Add 1 to each valid 0 point to differentiate from the later junk values
        local_map[local_map == 0] += 1

        # Rotate the local map based on the difference in angles of the transforms
        (h, w) = local_map.shape[:2]
        (origin_X, origin_Y) = (w / 2, h / 2)

        M = cv2.getRotationMatrix2D((origin_X, origin_Y), -rpy[2], 1.0)
        cos = np.abs(M[0, 0])
        sin = np.abs(M[0, 1])

        new_w = int((h * sin) + (w * cos))
        new_h = int((h * cos) + (w * sin))

        M[0, 2] += (new_w / 2) - origin_X
        M[1, 2] += (new_h / 2) - origin_Y
        adj_lmap = cv2.warpAffine(local_map, M, (new_w, new_h))

        # Add values from adjusted local map to world map

        (adj_y, adj_x) = adj_lmap.shape[:2]

        # dif = difference in the origins between local map and world map
        dif_x = int(100 * cur_tf.transform.translation.x)
        dif_y = int(100 * cur_tf.transform.translation.y)

        # corners of the local map relative to the full map
        corner_xU = dif_x - (adj_x / 2)
        corner_yL = dif_y - (adj_y / 2)
        corner_xD = corner_xU + adj_x
        corner_yR = corner_yL + adj_y

        # corners of the local map relative to itself
        lcorner_xU = 0
        lcorner_yL = 0
        lcorner_xD = adj_x
        lcorner_yR = adj_y

        # deals with values in local map that are in a row less than row 0
        if corner_xU < 0:
            lcorner_xU += (-corner_xU)
            corner_xU = 0

        # deals with values in local map that are in a column less than column 0
        if corner_yL < 0:
            lcorner_yL += (-corner_yL)
            corner_yL = 0

        (fY, fX) = self.map.shape[:2]  # max values of full map

        # deals with values in local map that are in a row greater than max row
        if corner_xD > fX:
            lcorner_xD -= (corner_xD - fX)
            corner_xD = fX

        # deals with values in local map that are in a column greater than max column
        if corner_yR > fY:
            lcorner_yR -= (corner_yR - fY)
            corner_yR = fY

        # Transfer all valid values out of adjust local map and update their corresponding positions in the Field
        mask = adj_lmap[lcorner_yL:lcorner_yR, lcorner_xU:lcorner_xD] != 0
        np.copyto(self.map[corner_yL:corner_yR, corner_xU:corner_xD],
                  adj_lmap[lcorner_yL:lcorner_yR, lcorner_xU:lcorner_xD], where=mask)

    def get_map_as_grid_msg(self):

        full_map = np.reshape(self.map, (1, self.map.shape[0] * self.map.shape[1])).astype(np.uint8)
        full_map = full_map.tolist()
        full_map = full_map[0]

        # Create OccupancyGrid messgae
        new_msg = OccupancyGrid()
        new_msg.data = full_map
        new_msg.info.resolution = 0.01
        new_msg.info.width = self.map.shape[1]
        new_msg.info.height = self.map.shape[0]
        new_msg.header.frame_id = "world"

        return new_msg


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("global_map_publisher")

    # Read parameters from server
    map_length = rospy.get_param("global_map_length", default=738)
    map_width = rospy.get_param("global_map_width", default=378)
    local_map_topic = rospy.get_param("local_elevation_map_topic", default="local_elevation_map")
    local_map_frame_id = rospy.get_param("local_elevation_map_frame_id", default="robot_center")
    global_map_frame_id = rospy.get_param("global_elevation_map_frame_id", default="world")

    # Create Field object
    field = Field(map_length, map_width, local_map_frame_id, global_map_frame_id)

    # Create subscriber to local map occupancy grid
    rospy.Subscriber(local_map_topic, OccupancyGrid, field.update_field)

    # Create publisher to publish global map
    global_map_pub = rospy.Publisher("global_elevation_map", OccupancyGrid, queue_size=0)

    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        global_map_pub.publish(field.get_map_as_grid_msg())
        loop_rate.sleep()
