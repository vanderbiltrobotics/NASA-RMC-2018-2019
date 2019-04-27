#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO


class AngleListener:
    def __init__(self, pinNumber):
        GPIO.setmode(GPIO.BOARD)
        #GPIO.setwarnings(False)
        self.p = GPIO.setup(pinNumber, GPIO.OUT)
        self.p = GPIO.PWM(pinNumber, 50)
        self.p.start(2.5)

    def setCameraAngle(self, theta):
        self.p.ChangeDutyCycle(theta.data / 18.0 + 2.5)


if __name__ == '__main__':
    rospy.init_node('camera_control_node', anonymous=True)
    listener = AngleListener(5)
    rospy.Subscriber("aruco/servo_theta", Int32, listener.setCameraAngle)
    rospy.spin()
