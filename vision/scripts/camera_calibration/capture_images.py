# --------------------------------------------------------------------------------------------- #
#
# This is a script for capturing images at a regular interval. It is intended to be used to
# capture images for camera calibration. The script shows the video stream for the specified
# camera in a window and saves images from the camera at regular intervals.
#
# Modified code from the OpenCV documentation found here:
# https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_gui/py_video_display/py_video_display.html
#
# --------------------------------------------------------------------------------------------- #

# Import required packages
import cv2
import time

# PARAMETERS TO SET
DEVICE_NUM = 0
SAVE_PATH = "./calibration_images/camera_a/"
CAPTURE_RATE = 3

# Create an object for video capture
stream = cv2.VideoCapture(DEVICE_NUM)

# Capture frames until we say to stop
frame_count = 0
start_time = time.time()
while(True):

    # Capture next frame
    ret, frame = stream.read()

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # If enough time has passed, save frame
    if time.time() - start_time > CAPTURE_RATE:
        cv2.imwrite(SAVE_PATH + "calib_img" + str(frame_count) + ".png", gray)
        print "Captured frame #" + str(frame_count)
        frame_count += 1
        start_time = time.time()

    # Display the frame
    cv2.imshow('frame', gray)

    # Check if we should quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Shut down
stream.release()
cv2.destroyAllWindows()