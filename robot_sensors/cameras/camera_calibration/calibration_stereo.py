

# Import required packages
import cv2
import os
import yaml
import numpy as np

# PARAMETERS FOR OUR SETUP
CB_COLS = 9
CB_ROWS = 7
CB_SQUARE_SIZE = 19
IMG_SIZE = (640, 480)
CALIBRATION_PATH_A = "./calibration_data/camera_a.yaml"
CALIBRATION_PATH_B = "./calibration_data/camera_b.yaml"
IMG_PATH_A = "./calibration_images/stereo/stereo_a/"
IMG_PATH_B = "./calibration_images/stereo/stereo_b/"
SAVE_PATH = "./calibration_data/stereo.yaml"
SAVE_PATH_MAPS = "./calibration_data/stereo_maps"

# termination criteria for finding sub-pixel corner positions
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Retrieve information from previous calibration steps
cal_data_a = yaml.load(open(CALIBRATION_PATH_A))
cal_data_b = yaml.load(open(CALIBRATION_PATH_B))

# load path names of images to be used for calibration - names should
# be the same for both directories so we only need to load one
fnames = os.listdir(IMG_PATH_A)

# list to store 'image points' - these are the coordinates of the corners that
# we find in the image using findChessBoardCorners.
image_points_a = []
image_points_b = []

# list to store 'object points' - these are the 'true' XYZ coordinates of the
# chessboard corners. Units are whatever we make them to be. The object points
# will be the same for all images. We compute them once, then use for every image
# Note: object points are the same for both images
op_single_img = np.zeros((CB_ROWS * CB_COLS, 3), np.float32)
op_single_img[:, 0:2] = np.mgrid[0:CB_ROWS, 0:CB_COLS].T.reshape(-1,2)
op_single_img = op_single_img * CB_SQUARE_SIZE
object_points = []

# Extract corner information from each of the images
for fname in fnames:

    # Load images, convert to grayscale
    img_a = cv2.imread(IMG_PATH_A + fname)
    img_b = cv2.imread(IMG_PATH_B + fname)
    gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY)

    # Find corners in the image
    ret_a, corners_a = cv2.findChessboardCorners(gray_a, (CB_ROWS, CB_COLS), None)
    ret_b, corners_b = cv2.findChessboardCorners(gray_b, (CB_ROWS, CB_COLS), None)

    # If corners found, continue, else skip this image, print a warning
    if ret_a and ret_b:

        # Improve corner accuracy
        better_corners_a = cv2.cornerSubPix(gray_a, corners_a, (11, 11), (-1, -1), criteria)
        better_corners_b = cv2.cornerSubPix(gray_b, corners_b, (11, 11), (-1, -1), criteria)

        # Display to make sure corner points are correct
        cv2.drawChessboardCorners(img_a, (CB_ROWS, CB_COLS), better_corners_a, ret_a)
        cv2.drawChessboardCorners(img_b, (CB_ROWS, CB_COLS), better_corners_b, ret_b)
        cv2.imshow('Image A', img_a)
        cv2.imshow('Image B', img_b)

        # Wait for response
        if cv2.waitKey(0) & 0xFF == ord('y'):

            # Add points to the full list as well as another copy of object points
            object_points.append(op_single_img)
            image_points_a.append(better_corners_a)
            image_points_b.append(better_corners_b)

            # Done with this frame
            print "-- processed " + fname + " --"

        # Otherwise, skip the frame
        else:
            print "-- skipped frame " + fname + " --"

    else:
        print "-- unable to find corners in " + fname + ", skipped frame --"

cv2.destroyAllWindows()

# Calibrate
cmatx_a = np.asarray(cal_data_a["camera_matrix"])
cmatx_b = np.asarray(cal_data_b["camera_matrix"])
dist_a = np.asarray(cal_data_a["dist_coefficients"])
dist_b = np.asarray(cal_data_b["dist_coefficients"])
ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(object_points, image_points_a, image_points_b, cmatx_a, dist_a,
                                                  cmatx_b, dist_b, IMG_SIZE, flags=cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_ZERO_TANGENT_DIST)

# Compute rectification matrices
R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(cmatx_a, dist_a, cmatx_b, dist_b, IMG_SIZE, R, T, alpha=0)

# Compute remapping matrices
map_a_x, map_a_y = cv2.initUndistortRectifyMap(cmatx_a, dist_a, R1, P1, IMG_SIZE, cv2.CV_32FC1)
map_b_x, map_b_y = cv2.initUndistortRectifyMap(cmatx_b, dist_b, R2, P2, IMG_SIZE, cv2.CV_32FC1)

# Save results
data = {"rotation_matrix": np.asarray(R).tolist(),
        "translation_vector": np.asarray(T).tolist(),
        "essential_matrix": np.asarray(E).tolist(),
        "fundamental_matrix": np.asarray(F).tolist(),
        "R1": np.asarray(R1).tolist(),
        "R2": np.asarray(R2).tolist(),
        "P1": np.asarray(P1).tolist(),
        "P2": np.asarray(P2).tolist(),
        "Q": np.asarray(Q).tolist(),
        "roi_1": np.asarray(roi1).tolist(),
        "roi_2": np.asarray(roi2).tolist()}
yaml.dump(data, open(SAVE_PATH, 'w'))
np.savez_compressed(SAVE_PATH_MAPS, ax=map_a_x, ay=map_a_y, bx=map_b_x, by=map_b_y)

# Testing
test_a = cv2.imread(IMG_PATH_A + fnames[0])
test_b = cv2.imread(IMG_PATH_B + fnames[0])
cv2.imshow("Original A", test_a)
cv2.imshow("Original B", test_b)

remapped_a = cv2.remap(test_a, map_a_x, map_a_y, cv2.INTER_LINEAR)
remapped_b = cv2.remap(test_b, map_b_x, map_b_y, cv2.INTER_LINEAR)
cv2.imshow("Remapped A", remapped_a)
cv2.imshow("Remapped B", remapped_b)

cv2.waitKey(0)
