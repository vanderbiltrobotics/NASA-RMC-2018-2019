import numpy as np
from is_clear_path import is_clear_path

''' 

This program attempts to find an "easy" path through the field. It takes the driveability map,
maximum and minimum y-coordinates, and maximum x-offset as parameters. The min and max y-coords are 
the ideal start and end points, respectively. The algorithm first computes the path between the ideal 
coords and checks if the path is clear. If it is not, it adds a predefined x-offset until either a clear 
path is found, or the field boundary is reached. If there is no clear path, the original points are returned.

'''


# Compute a simple path along the hypotenuse between the start and end points
# Contains start point in path
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

    # Prepend starting point
    path = np.insert(path, 0, start_pos, axis=0)
    return path


# driveability_map is in cm, y_min and y_max are in m, max_x_offset is in cm
def compute_obs_region_endpoints(driveability_map, y_min, y_max, max_x_offset):
    field_width = driveability_map.shape[1]

    # Starting p1 and p2; Converted to cm
    # Must retain original points in case no easy path is found
    p1_orig = (y_min * 100.0, field_width / 2.0)
    p2_orig = (y_max * 100.0, field_width / 2.0)

    # Rounded p1 and p2; Converted to cm
    # In case occupancy grid is odd
    p1 = (y_min * 100.0, np.rint(field_width / 2.0))
    p2 = (y_max * 100.0, np.rint(field_width / 2.0))

    # Left and right p1 and p2
    p1_left = (0, 0)
    p2_left = (0, 0)
    p1_right = (0, 0)
    p2_right = (0, 0)

    # Initialize to first enter while loop
    left_offset = -1  # cm; Becomes 0 right away in loop
    right_offset = -1
    clear_path_left = False  # Need two paths in case occupancy grid has odd width
    clear_path_right = False

    # Search until clear path is found or field boundaries are reached
    while not clear_path_left and not clear_path_right and left_offset <= max_x_offset \
            and right_offset <= max_x_offset+1:
        # Increase offset
        right_offset += 1
        left_offset += 1

        # Add offset in both x-directions to avoid obstacles
        p1_right = (p1[0], p1[1] + right_offset)
        p1_left = (p1[0], p1[1] - left_offset)
        p2_right = (p2[0], p2[1] + right_offset)
        p2_left = (p2[0], p2[1] - left_offset)

        # Computes easy path in both directions
        # Step size is the same as for driveability_map
        # Points and step size in cm
        path_right = compute_easy_path(p1_right, p2_right, step_size=1)
        path_left = compute_easy_path(p1_left, p2_left, step_size=1)

        # check if p1 and p2 line up in a clear path
        # Path is sent in rounded cm to be compatible with driveability_map
        clear_path_right = is_clear_path(driveability_map, (path_right+0.5).astype(int))
        clear_path_left = is_clear_path(driveability_map, (path_left+0.5).astype(int))

    # Return points in clear path, w/ preference to the right
    # Return starting points if no path found
    if clear_path_right:
        return p1_right, p2_right
    elif clear_path_left:
        return p1_left, p2_left
    else:
        return p1_orig, p2_orig


if __name__ == "__main__":
    # Odd array to test
    # test = np.array(
    #     [[0, 0, 0, 0, 0, 1, 0],
    #      [0, 0, 0, 0, 0, 0, 0],
    #      [1, 0, 1, 1, 1, 0, 0],
    #      [0, 0, 0, 0, 0, 0, 0],
    #      [0, 1, 0, 0, 0, 0, 0],
    #      [0, 0, 1, 0, 0, 0, 0]])

    # Even array to test
    test = np.array(
        [[0, 0, 0, 0, 0, 1, 1, 1],
         [0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 1, 1, 1, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0, 0, 0],
         [0, 0, 1, 0, 0, 0, 0, 0]])
    p1, p2 = compute_obs_region_endpoints(test, 0, .05, 3)
    print(p1)
    print (p2)
