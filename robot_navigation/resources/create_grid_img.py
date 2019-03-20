
import numpy as np
import cv2


# Parameters
WIDTH = 3.81
LENGTH = 7.31
BIN_WIDTH = 0.48
BIN_LENGTH = 1.65
BIN_Y_OFFSET = 0.5
RESOLUTION = 0.01
FILENAME = "empty_arena_1cm.bmp"

# --- CREATE IMAGE --- #

multiplier = int(1.0 / RESOLUTION)

# Create base image
img = np.zeros((int(LENGTH * multiplier), int(WIDTH * multiplier)))

# Add collection bin
bin_y_start = int((LENGTH - BIN_Y_OFFSET - BIN_LENGTH) * multiplier)
bin_y_end = int((LENGTH - BIN_Y_OFFSET) * multiplier)
bin_x_end = int(BIN_WIDTH * multiplier)
img[bin_y_start: bin_y_end, 0: bin_x_end] = 255

# Save the image
cv2.imwrite("../resources/" + FILENAME, img)
