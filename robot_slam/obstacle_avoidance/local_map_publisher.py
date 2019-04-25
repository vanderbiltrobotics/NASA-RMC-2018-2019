import rospy
import numpy as np
import tf2_ros
import cv2
from nav_msgs.msg import OccupancyGrid


class LocalMapper:

    def __init__(self):

        # Local Occupancy Grid
        self.grid_msg = OccupancyGrid()

    def update_local_map(self, new_arr):

        # Flatten numpy array
        map_data = np.reshape(new_arr, (1, new_arr.shape[0] * new_arr.shape[1])).astype(np.uint8).tolist()
        map_data = map_data[0]

        # Convert numpy array to occupancy grid
        new_local_map = OccupancyGrid()
        new_local_map.data = map_data
        new_local_map.header.frame_id = "robot_center"
        new_local_map.info.resolution = 0.01
        new_local_map.info.width = new_arr.shape[1]
        new_local_map.info.height = new_arr.shape[0]

        # Update the map
        self.grid_msg = new_local_map

if __name__ == "__main__":

    # Initialize ros node
    rospy.init_node('local_map_publisher')

    # Define numpy array for local map
    local_map = np.ones(shape=(50, 100)) * 100

    # Create LocalMapper object
    mapper = LocalMapper()

    # Update with our map
    mapper.update_local_map(local_map)

    # Periodically publish the map message
    map_pub = rospy.Publisher("local_elevation_map", OccupancyGrid, queue_size=0)
    loop_rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        map_pub.publish(mapper.grid_msg)
        loop_rate.sleep()




