#!/usr/bin/env python

# Receives ROS messages about if an Aruco marker has been detected and the average
# x location of its corners. Centers the camera using a P feedback loop on this x-coordinate.

# Import ROS packages
import rospy
import rospkg 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import tf.transformations

# Import other required packages
import cv2.aruco as aruco
import numpy as np

class ArucoCamera:

    def __init__(self):

        # Initialize an Aruco camera object

        self.cameraCenterX = rospy.get_param("image_width", default=640) / 2  # Camera center 
        self.theta = Int32()  # ROS object that stores current servo theta orientation
        self.error = 0        # Error between current theta and where the camera should be 
        self.markerCenter = 0 # Stores the center position of the marker
        self.minTheta = -90   # Minimum servo bound
        self.maxTheta = 90    # Maximum servo bound
        self.markerDetected = False # True if an AruCo marker is in view
        self.sweepDirection = 1 # Controls the direction of sweep

        # Proportional gain; It represents a transformation from the camera pixel subspace to servo angle. 
        # Vary the magnitude of k to change how much the servo angle should change based off the error
        self.k = -0.0005 

        
        self.deltaTheta = 1
        self.rangeOfMax = 2

        self.servoThetaPub = rospy.Publisher("aruco/servo_theta", Int32, queue_size=0)

    # Decides whether to track Aruco marker or sweep camera
    def updateMarkerDetected(self,detected):
        self.markerDetected = detected.data

    # Update the x position of the Aruco marker
    def updateSetPoint(self,markerCenter):
        self.markerCenter = markerCenter.data
    
    # Rotate servo to track the Aruco marker or sweep the camera to find a marker
    def updatePosition(self):
        if self.markerDetected:
            self.error = self.markerCenter - self.cameraCenterX  # servo will track Aruco marker
            self.k = -0.0005
        else:
            
            # Update set point
            self.error += self.deltaTheta*self.sweepDirection
            self.k = 1

        # Calculate new servo position
        self.theta.data += self.k * self.error
      
        # Check servo bounds and switch sweep direction
        if self.theta.data >= self.maxTheta:
            self.theta.data = self.maxTheta
            self.sweepDirection = -1

        elif self.theta.data <= self.minTheta:
            self.theta.data = self.minTheta
            self.sweepDirection = 1

        # Update servo position
        self.servoThetaPub.publish(self.theta)


if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node('aruco_camera')

    # Initialize ArucoCamera object
    arucoCamera = ArucoCamera()

    # Create subscribers for detecting an Aruco board and obtaining the avg x-val of corners
    markerDetectedSub = rospy.Subscriber("aruco/marker_detected", Bool, arucoCamera.updateMarkerDetected)
    avgCornerSub = rospy.Subscriber("aruco/avg_of_corners", Int32, arucoCamera.updateSetPoint)

    # Set loop rate
    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        arucoCamera.updatePosition()
        r.sleep()
