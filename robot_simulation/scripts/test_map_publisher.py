
# Import ROS packages
import rospy
from nav_msgs.msg import OccupancyGrid

# Import other required packages
import numpy as np
from random import randint, gauss
from math import sqrt


def generate_map(length, width):

    # First generate empty map
    new_map = np.zeros((length, width), np.uint8)

    # Obstacles can only be added between these bounds
    upper_bound = length / 3
    lower_bound = upper_bound * 2
    num_obs = 5
    min_obs_size = 10
    max_obs_size = 75

    # Add obstacles randomly throughout grid
    for i in range(num_obs):

        # Randomly generate center coords of object
        center_x = randint(max_obs_size / 2, width - max_obs_size / 2)
        center_y = randint(upper_bound + (max_obs_size / 2), lower_bound - (max_obs_size / 2))

        # Randomly generate size of the object
        mu = (min_obs_size + max_obs_size) / 2.0
        sigma = (max_obs_size - min_obs_size) / 6.0
        obs_size = gauss(mu, sigma)

        # Limit object size
        obs_size = min_obs_size if obs_size < min_obs_size else obs_size
        obs_size = max_obs_size if obs_size > max_obs_size else obs_size
        obs_rad = obs_size / 2.0

        # Add obstacle to map
        for x in range(0, width):
            for y in range(length):
                if sqrt(pow((x - center_x), 2) + pow((y - center_y), 2)) < obs_rad:
                    new_map[y, x] = 1

    return new_map


# Run the node
if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("test_map_publisher")

    # Create publisher for map
    map_pub = rospy.Publisher("test_map", OccupancyGrid, queue_size=0)

    length = 731
    width = 381

    # Create numpy array representing map
    map = generate_map(length, width)

    # Convert map to an OccupancyGrid message
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = "world"
    map_msg.info.resolution = 0.01
    map_msg.info.height = length
    map_msg.info.width = width
    map_msg.data = np.reshape(map, (1, length * width)).tolist()[0]

    # Periodically publish the map
    loop_rate = rospy.Rate(1.0)

    while not rospy.is_shutdown():

        # Publish map
        map_pub.publish(map_msg)

        # Sleep
        loop_rate.sleep()







