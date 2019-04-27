#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO


def setCameraAngle(theta):
    p.changeDutyCycle((theta/18.0) + 2.5)

def angleListener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_control_node', anonymous=True)

    rospy.Subscriber("aruco/servo_theta", Int32, setCameraAngle)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)

    #set output pin to pin 12
    GPIO.setup(12, GPIO.OUT)

    #pwm on pin 12 set at 50Hz
    p = GPIO.PWM(12, 50)
    p.start(2.5) #initialization
    angleListener