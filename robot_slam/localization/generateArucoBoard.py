import cv2.aruco as aruco
import numpy as np

# Simple script to generate a specific Arubo board. Writes the board to a file.

# Parameters to vary
numMarkersX = 0
numMarkersY = 0
markerLength = 0
markerSeparation = 0

# Create dictionary and board
dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()  # default parameters
board = aruco.GridBoard_create(numMarkersX, numMarkersY, markerLength, markerSeparation, dictionary)

# Draw board and write to file
boardImage = aruco.draw(board, boardImage)
