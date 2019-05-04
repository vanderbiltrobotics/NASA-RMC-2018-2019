#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO


class AngleListener:

    def __init__(self, pinNumber, max_angle):

        # Set the pin as an output
        self.p = GPIO.setup(pinNumber, GPIO.OUT)
        self.p = GPIO.PWM(pinNumber, 1000)

        # Store max angle to use for scaling
        self.max_angle = max_angle

        # Start
        self.p.start(50)

    def setCameraAngle(self, theta):

        # Convert from range -90 to 90 to range 0 to 100
        duty_cycle = ((theta.data + self.max_angle / (self.max_angle / 2))) * 100

        # Send new duty cycle to servo
        self.p.ChangeDutyCycle(duty_cycle)


if __name__ == '__main__':

    # Initialize GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    # Initialize ROS node
    rospy.init_node('camera_control_node', anonymous=True)

    # Create listener object
    listener = AngleListener(18, 90.0)

    # Attach subscriber to servo angle topic
    rospy.Subscriber("aruco/servo_theta", Int32, listener.setCameraAngle)

    # Spin
    rospy.spin()
