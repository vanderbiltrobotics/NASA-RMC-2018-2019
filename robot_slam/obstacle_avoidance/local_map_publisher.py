import rospy
import numpy as np
import tf2_ros
import cv2

class LocalMapper:
    def __init__(self):
        self.grid_msg = OccupancyGrid
        self.map_pub = rospy.Publisher("local_map", OccupancyGrid, map_callback)

    def map_callback(self):
        self.grid_msg.data = np.array(length = 100, width = 50)
        self.grid_msg.data.fill(self, 1)

        self.map_map.publish(grid_msg)

if __name__ == "__main__":
    rospy.init_node('local_map_publisher')


