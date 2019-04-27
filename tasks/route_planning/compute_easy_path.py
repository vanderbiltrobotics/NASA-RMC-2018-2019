import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header


def compute_easy_path(start_pos, end_pos, step_size):
    # calculating length of the straight line and the number of points
    dy = end_pos[1] - start_pos[1]
    dx = end_pos[0] - start_pos[0]
    path_length = np.sqrt(dx ** 2 + dy ** 2)
    matrix_len = int(path_length / step_size)

    # creating matrices for calculations
    points = [start_pos for i in range(matrix_len)]
    steps = [[step_size * dx / path_length, step_size * dy / path_length] for i in range(matrix_len)]
    nums = [[i + 1, i + 1] for i in range(matrix_len)]

    # Calculating the path
    path = np.add(points, np.multiply(steps, nums))

    # returning ROS Path object
    return Path(
        # setting poses equal to an array of PoseStamped messages
        poses=[PoseStamped(
            # pose of each PoseStamped equal to the x and y of the point
            pose=Pose(
                position=Point(
                    x=point[0],
                    y=point[1]
                )
            ),
            # header of each PoseStamped equal to world
            header=Header(frame_id='world')
        # for each point in the path
        ) for point in path]
    )
