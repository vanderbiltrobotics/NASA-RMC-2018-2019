import numpy as np

''' 

This program checks if a given path is clear of obstacles. It does so by checking the occupancy grid
for each point along the path for obstacles. If none are found, it returns True, otherwise False.

'''


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


if __name__ == "__main__":
    # Just for testing this value in a vaccuum
    test = np.array(
        [[0, 0, 0, 0, 0, 1],
         [0, 0, 0, 0, 1, 0],
         [0, 0, 0, 1, 0, 0],
         [0, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0],
         [1, 0, 0, 0, 1, 0]], np.int32)
