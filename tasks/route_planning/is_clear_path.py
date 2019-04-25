import numpy as np
import cv2

def is_clear_path(driveability_map, start_pos, end_pos):
    # The map contains all the points we can move across; must avoid obstacles

    #Get two points, and find the straightest line possible between them. Store these points in another array.
    #Raw inputs for ease of testing
    #x1 = int(raw_input('X1 coordinate: '))
    #y1 = int(raw_input('Y1 coordinate: '))
    #x2 = int(raw_input('X2 coordinate: '))
    #y2 = int(raw_input('Y2 coordinate: '))

    #Actual inputs
    x1, y1 = start_pos
    x2, y2 = end_pos
    dif_x = x2 - x1
    dif_y = y2 - y1

    if dif_x == 0:
        step = np.sign(dif_y)
        ys = np.arange(0, dif_y + step, step)
        xs = np.repeat(x1, ys.shape[0])

    else:
        m = dif_y/(dif_x+0.0)
        b = y1 - m * x1

        step = 1.0/(max(abs(dif_x),abs(dif_y)))
        xs = np.arange(x1, x2, step * np.sign(x2 - x1))
        ys = xs * m + b

    xs = np.rint(xs)
    ys = np.rint(ys)
    pts_along_line = np.column_stack((xs, ys))

    # Due to rounding (array elements must be int), this helps take care of
    # redundant points and speed things up a bit

    # Perform lex sort and get sorted data
    sorted_idx = np.lexsort(pts_along_line.T)
    sorted_data =  pts_along_line[sorted_idx,:]

    # Get unique row mask
    row_mask = np.append([True],np.any(np.diff(sorted_data,axis = 0),1))

    # Get unique rows
    pts_along_line = sorted_data[row_mask]


    # If we find a value greater than a certain threshold, make a note of that.
    # That means there's an obstacle in the way and we can drive directly through this line.
    for pt in pts_along_line:
        y, x = pt
        x = int(x)
        y = int(y)
        occupied = driveability_map[x, y]
        if occupied == 1:
            return False # obstacle detected, can't cross here

    return True; # Safe to drive along this line

if __name__ == "__main__":
    # Just for testing this value in a vaccuum
    test = np.array(
        [[0, 0, 0, 0, 0, 1],
         [0, 0, 0, 0, 1, 0],
         [0, 0, 0, 1, 0, 0],
         [0, 0, 0, 0, 0, 0],
         [0, 1, 0, 0, 0, 0],
         [1, 0, 0, 0, 1, 0]], np.int32)
    print(is_clear_path(test, (2,0), (2,5)))
