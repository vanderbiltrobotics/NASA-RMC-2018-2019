# Implements an EKF algorithm from the FilterPy library for the Aruco Marker data input

from filterpy.kalman import EKF
import numpy as np

# Initialize the Extended Kalman Filter Object
# First argument is dimension of the state vector X
# For AruCo, our state vector consists of (x,y,theta)
# Second arugment is dimension of the measurement input Z
# For AruCo, measurement input is (x,y,theta)
arucoEKF = ExtendedKalmanFilter(3,3)

# Define the measurement noise covariance matrix
#arucoEKF.R = 

# Define the process noise covariance matrix
#arucoEKF.Q = 

# Define the state transition matrix
#arucoEKF.F = 

# Define the Jacobian matrix for measurement update
#H = 

# Define callback function to calculate the expected sensor measurement
#def hx():
