
# A-star path finding library
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

# ROS packages
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header

# Other important packages
import numpy as np


# Check if input point is within the field boundaries
def is_in_field(driveability_map, pt):
    return True if ((pt >= 0).all() and pt[0] < driveability_map.shape[0] and pt[1] < driveability_map.shape[1]) \
        else False


# Check occupancy grid for obstacles along calculated path
# Takes path as type int to be compatible with driveability_map
def is_clear_path(driveability_map, path):

    # Check for obstacles
    for pt in path:

        # Check boundaries
        if not is_in_field(driveability_map, pt):
            return False

        occupied = driveability_map[pt[0], pt[1]]
        if occupied == 1:
            return False  # obstacle detected, can't cross here

    return True  # Safe to drive along this line


# Generates a ROS Path message for a path between two points - assumes that
# there are no obstacles on the path. Points on the path separated by step_size
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


# Computes a 'difficult' path between two points, provided with an obstacle map - does NOT assume
# that there are no obstacles between the points.
# K = resolution reduction factor to apply - reduces computation time significantly
def compute_hard_path(map, K):
    # First, trim the map down such that it can be divided evenly into K by K square sections.
    # Try to keep the trimming as symmetric as possible: If we trim the bottom side, trim the top side next, etc.
    H, W = map.shape
    K = K

    H_excess = H % K
    W_excess = W % K

    start_x = H_excess / 2
    end_x = H - (H_excess / 2)

    start_y = W_excess / 2
    end_y = W - (W_excess / 2)

    # In the event that we only need to trim one edge to make that dimension divisible by K, we have over-adjusted
    # in the above code. Rectify that here - is there a simple way to not make that mistake prior?
    if (H_excess % 2 == 1):
        end_x -= 1

    if (W_excess % 2 == 1):
        end_y -= 1

    map = map[start_x:end_x, start_y:end_y]  # Adjusted map that can now be divided into KxK sections

    # Divide the adjusted map into KxK sections, taking the max value of each section to be the value of that
    # section.
    # We can also take a running total of the number of 1's in each section, to determine which
    # sections are least likely to be impassable.
    HK = H // K
    WK = W // K

    weighted_map = (map[:HK * K, :WK * K].reshape(HK, K, WK, K).sum(axis=(1, 3)))

    print 'Weighted reduced map:'
    print weighted_map

    weighted_map[weighted_map > 0] *= -1
    weighted_map[weighted_map == 0] = 1

    grid = Grid(matrix=weighted_map)
    start = grid.node(2, 0)
    end = grid.node(0, 2)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    path, runs = finder.find_path(start, end, grid)

    path_found = (len(path) != 0)
    threshold = 0
    while not path_found:
        threshold -= 1
        weighted_map[weighted_map == threshold] = 1
        grid = Grid(matrix=weighted_map)
        start = grid.node(2, 0)
        end = grid.node(0, 2)

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
        path, runs = finder.find_path(start, end, grid)
        path_found = (len(path) != 0)

    print(path)
    print('operations:', runs, 'path length:', len(path))
    print(grid.grid_str(path=path, start=start, end=end))
    print 'Highest weight allowed to drive over: ', threshold * -1

    adj_path = np.array(path)
    adj_path = K * adj_path + (K/2)
    print adj_path

    for pt in adj_path[:-1]:
        # computeEasyPath(pt, pt + 1, stepSize?????)
        print('hey') #placeholder so the red squiggly leaves me alone

    return weighted_map


if __name__ == "__main__":
    map = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 0, 0, 0, 0, 0, 0, 1],
                    [1, 0, 1, 1, 0, 0, 0, 0, 1],
                    [1, 1, 1, 1, 0, 0, 0, 0, 0],
                    [1, 0, 0, 1, 0, 0, 0, 0, 0],
                    [1, 0, 0, 1, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 1, 0, 1],
                    [0, 0, 0, 0, 0, 1, 0, 0, 1],
                    [0, 0, 0, 1, 0, 0, 0, 0, 1]])

    K = 3

    max_pool(map, K)



