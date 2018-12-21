# --------------------------------------------------------------------------------------------- #
#
# This is a script for capturing images at a regular interval. It is intended to be used to
# capture images for camera calibration. The script shows the video stream for the specified
# camera in a window and saves images from the camera at regular intervals.
#
# Modified code from the OpenCV documentation found here:
# https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_gui/py_video_display/py_video_display.html
#
# Usage instructions:
# Every [CAPTURE_RATE] seconds, the current frame will be captured - if chessboard corners are
# found, they will be drawn on the image. If you wish to save the image, press 'y'. Otherwise,
# press any other key or wait [PREVIEW_TIME] seconds and the frame will expire. Press 'q' to quit
#
# --------------------------------------------------------------------------------------------- #

# Import required packages
import cv2
import time

# PARAMETERS TO SET
DEVICE_NUM_A = 2
DEVICE_NUM_B = 1
SAVE_PATH_A = "./calibration_images/stereo/stereo_a/"
SAVE_PATH_B = "./calibration_images/stereo/stereo_b/"
CAPTURE_RATE = 5
PREVIEW_TIME = 10
CB_COLS = 9
CB_ROWS = 7

# Create an object for video capture
stream_a = cv2.VideoCapture(DEVICE_NUM_A)
stream_b = cv2.VideoCapture(DEVICE_NUM_B)

# Capture frames until we say to stop
frame_count = 0
start_time = time.time()
while True:

    # Capture next frame
    _, frame_a = stream_a.read()
    _, frame_b = stream_b.read()

    # Convert to grayscale
    gray_a = cv2.cvtColor(frame_a, cv2.COLOR_BGR2GRAY)
    gray_b = cv2.cvtColor(frame_b, cv2.COLOR_BGR2GRAY)

    # If enough time has passed, save frame
    if time.time() - start_time > CAPTURE_RATE:

        # Try to find chessboard corners in the image, draw them on the image
        ret_a, corners_a = cv2.findChessboardCorners(frame_a, (CB_ROWS, CB_COLS), None)
        ret_b, corners_b = cv2.findChessboardCorners(frame_b, (CB_ROWS, CB_COLS), None)
        cv2.drawChessboardCorners(frame_a, (CB_ROWS, CB_COLS), corners_a, ret_a)
        cv2.drawChessboardCorners(frame_b, (CB_ROWS, CB_COLS), corners_b, ret_b)

        # Show the frame, wait for user to choose to keep the image or not
        cv2.imshow("frame_a", frame_a)
        cv2.imshow("frame_b", frame_b)

        if cv2.waitKey(PREVIEW_TIME * 1000) & 0xFF == ord('y'):

            # Frame is good, save it, increment frame count
            cv2.imwrite(SAVE_PATH_A + "calib_img" + str(frame_count) + ".png", gray_a)
            cv2.imwrite(SAVE_PATH_B + "calib_img" + str(frame_count) + ".png", gray_b)
            print "Captured frame #" + str(frame_count)
            frame_count += 1

        # Reset timer
        start_time = time.time()

    # Display the frame
    cv2.imshow('frame_a', frame_a)
    cv2.imshow('frame_b', frame_b)

    # Check if we should quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Shut down
stream_a.release()
stream_b.release()
cv2.destroyAllWindows()