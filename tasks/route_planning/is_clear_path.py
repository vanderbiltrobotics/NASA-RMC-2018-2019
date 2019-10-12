import numpy as np

''' 

This program checks if a given path is clear of obstacles. It does so by checking the occupancy grid
for each point along the path for obstacles. If none are found, it returns True, otherwise False.

'''


# Check if input point is within the field boundaries
def is_in_field(driveability_map, pt):
    return True if ((pt >= 0).all() and pt[0] < driveability_map.shape[0] and pt[1] < driveability_map.shape[1]) \
        else False


# Checks if the path between two points is clear
# Drivability map is in cm
# Start / end pos are in meters
def is_clear_path(driveability_map, start_pos, end_pos):

    # Convert start and end positions to cm
    start_pos = [int(i * 100) for i in start_pos]
    end_pos = [int(i * 100) for i in end_pos]

    # Set step size to 1 so we don' miss any obstacles
    step_size = 1

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

    # Prepend starting point
    path = np.insert(path, 0, start_pos, axis=0)

    # Check for obstacles
    for pt in path:

        # Check boundaries
        if not is_in_field(driveability_map, pt):
            return False

        occupied = driveability_map[pt[0], pt[1]]
        if occupied == 1:
            return False  # obstacle detected, can't cross here

    return True  # Safe to drive along this line


if __name__ == "__main__":
    # Just for testing this value in a vaccuum
    test = np.array(
        [[0, 0, 0, 0, 0, 1],
         [0, 0, 0, 0, 1, 0],
         [0, 0, 0, 1, 0, 0],
         [0, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0],
         [1, 0, 0, 0, 1, 0]], np.int32)
