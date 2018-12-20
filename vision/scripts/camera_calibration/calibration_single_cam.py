# --------------------------------------------------------------------------------------------- #
#
# This is a script for calibrating a single camera. It requires that you first have taken a
# series of calibration images using a chessboard pattern. All of the parameters at the top
# should be set according to your particular usage before running the script. For background,
# see the following links:
#
# https://docs.opencv.org/3.2.0/dc/dbb/tutorial_py_calibration.html
# https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
#
# The code explained in these links was modified to create this script.
#
# --------------------------------------------------------------------------------------------- #

# Import required packages
import cv2
import os
import yaml
import numpy as np

# PARAMETERS FOR OUR SETUP - MAKE SURE THESE ARE CORRECT
CB_COLS = 9
CB_ROWS = 7
CB_SQUARE_SIZE = 19
IMG_SIZE = (480, 640)
OPEN_PATH = "./calibration_images/jake_desktop/"
SAVE_PATH = "./calibration_data/jake_desktop.yaml"

# termination criteria for finding sub-pixel corner positions
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# load path names of images to be used for calibration
fnames = os.listdir(OPEN_PATH)

# list to store 'image points' - these are the coordinates of the corners that
# we find in the image using findChessBoardCorners.
image_points = []

# list to store 'object points' - these are the 'true' XYZ coordinates of the
# chessboard corners. Units are whatever we make them to be. The object points
# will be the same for all images. We compute them once, then use for every image
op_single_img = np.zeros((CB_ROWS * CB_COLS, 3), np.float32)
op_single_img[:, 0:2] = np.mgrid[0:CB_ROWS, 0:CB_COLS].T.reshape(-1,2)
op_single_img = op_single_img * CB_SQUARE_SIZE
object_points = []

# Extract corner information from each of the images
for fname in fnames:

    # Load image, convert to grayscale
    img = cv2.imread(OPEN_PATH + fname, cv2.IMREAD_GRAYSCALE)

    # Find corners in the image
    ret, corners = cv2.findChessboardCorners(img, (CB_ROWS, CB_COLS), None)

    # If corners found, continue, else skip this image, print a warning
    if ret:
        object_points.append(op_single_img)
        cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
        image_points.append(corners)
        print "-- processed " + fname + " --"
    else:
        print "Unable to find corners in " + fname

# Calibrate using the corner data
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, IMG_SIZE, None, None)

# Save calibration camera matrix and distortion coefficients to file for later use
data = {"camera_matrix": mtx, "dist_coeffictients": dist}
print "\n=========\n RESULTS\n=========\n"
print "Dist. Coeffifients: "
print dist
print "\nCamera Matrix"
print mtx
print "\nData saved to " + SAVE_PATH

out = open(SAVE_PATH, 'w')
yaml.dump(data, out)
