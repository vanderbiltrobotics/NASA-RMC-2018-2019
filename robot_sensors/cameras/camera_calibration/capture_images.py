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
DEVICE_NUM = 1
SAVE_PATH = "./calibration_images/camera_b/"
CAPTURE_RATE = 5
PREVIEW_TIME = 10
CB_COLS = 9
CB_ROWS = 7

# Create an object for video capture
stream = cv2.VideoCapture(DEVICE_NUM)

# Capture frames until we say to stop
frame_count = 0
start_time = time.time()
while True:

    # Capture next frame
    ret, frame = stream.read()

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # If enough time has passed, save frame
    if time.time() - start_time > CAPTURE_RATE:

        # Try to find chessboard corners in the image, draw them on the image
        ret, corners = cv2.findChessboardCorners(frame, (CB_ROWS, CB_COLS), None)
        cv2.drawChessboardCorners(frame, (CB_ROWS, CB_COLS), corners, ret)

        # Show the frame, wait for user to choose to keep the image or not
        cv2.imshow("frame", frame)
        if cv2.waitKey(PREVIEW_TIME * 1000) & 0xFF == ord('y'):

            # Frame is good, save it, increment frame count
            cv2.imwrite(SAVE_PATH + "calib_img" + str(frame_count) + ".png", gray)
            print "Captured frame #" + str(frame_count)
            frame_count += 1

        # Reset timer
        start_time = time.time()

    # Display the frame
    cv2.imshow('frame', frame)

    # Check if we should quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Shut down
stream.release()
cv2.destroyAllWindows()