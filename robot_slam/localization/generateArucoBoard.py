import cv2 
import cv2.aruco as aruco
import numpy as np

# Simple script to generate a specific Arubo board. Writes the board to a file.

# Parameters to vary
numMarkersX = 5
numMarkersY = 7
markerLength = .029
markerSeparation = .006

# Create dictionary and board
dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters_create()  # default parameters
board = aruco.GridBoard_create(numMarkersX, numMarkersY, markerLength, markerSeparation, dictionary)

# Draw board and write to file
image = board.draw((1000, 1000))
cv2.imwrite('arucoBoard.png',image)
