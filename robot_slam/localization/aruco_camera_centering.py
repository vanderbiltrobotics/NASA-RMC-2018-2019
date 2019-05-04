#!/usr/bin/env python

# This program keeps a camera mounted to a servo focused on an AruCo marker by rotating the servo in the 
# required direction as the AruCo marker position changes. It receives ROS messages about if an Aruco marker has been detected and 
# the average x location of its corners. Centers the camera using a Proportional gain feedback loop. When there is no AruCo marker 
# in sight, the servo will sweep from the minimum bound to maximum bound and back until it finds an AruCo marker.

# Alex Barnett, 2019

# Import ROS packages
import rospy
import rospkg 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import tf.transformations
from math import radians

# Import other required packages
import cv2.aruco as aruco
import numpy as np

# Class with internal state defining required camera dimensions and knowledge about the position of 
# the Aruco marker being detected.
class ArucoCamera:

    def __init__(self):
        
        # Camera dimensions
        self.cameraCenterX = rospy.get_param("image_width", default=640) / 2  # Camera center 

        # Servo state
        self.theta = Int32()         # ROS object that stores current servo theta orientation
        self.error = 0               # Error between current theta and where the camera should be 
        self.minTheta = -(135.0 / 2)          # Minimum servo bound
        self.maxTheta = (135.0 / 2)           # Maximum servo bound

        # Marker paramters
        self.markerDetected = False  # True if an AruCo marker is in view
        self.markerCenter = 0        # Stores the center position of the marker
        
        # Camera sweep parameters
        self.sweepDirection = 1      # Controls the direction of sweep
        self.deltaTheta = 1          # Servo angle change per loop during camera sweep

        # Proportional gain; represents a transformation from the camera pixel subspace to servo angle. 
        # Vary the magnitude of k to change how much the servo angle should change based off the error
        self.k = -0.0125

        # Publish servo theta 
        self.servoThetaPub = rospy.Publisher("aruco/servo_theta", Int32, queue_size=0)

        # Publish the pose transform
        self.lensPosePub = rospy.Publisher("camera_mount_pose", Pose, queue_size=0)

    # Update if there is an AruCo marker in the camera's FOV
    def updateMarkerDetected(self,detected):
        self.markerDetected = detected.data

    # Update the x-coordinate of the pixel of the AruCo marker center. Stores marker center relative
    # to the center of the camera
    def updateSetPoint(self,markerCenter):
        self.markerCenter = markerCenter.data - self.cameraCenterX
    
    # Rotate servo to track the Aruco marker or sweep the camera until a marker is found
    def updatePosition(self):
        if self.markerDetected:
            self.error = self.markerCenter  # Find error relative to camera center
        else:
            # Will increment servo by deltaTheta in the sweep direction
            # Divides by P gain so that the P gain will cancel out in new theta calculation
            self.error = self.deltaTheta*self.sweepDirection / self.k

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

        # Update pose transform
        camera_to_mount = Pose()
        # TODO Change vals
        camera_to_mount.position.x = 0
        camera_to_mount.position.y = 0
        camera_to_mount.position.z = 0
        camera_to_mount.orientation.z = tf.transformations.quaternion_from_euler(0, 0, radians(self.theta.data))[3]
        self.lensPosePub.publish(camera_to_mount)


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

    # Run continuously
    while not rospy.is_shutdown():
        arucoCamera.updatePosition()
        r.sleep()
