#!/usr/bin/env python

# ----------------------------- #
# Outputs some fake velocities to test drive node #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist

# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('test_node')

    # Ready to go
    rospy.loginfo("Drive Motor Controller initialized...")

    # Publisher for velocities
    test_publisher = rospy.Publisher('drive_cmd', Twist, queue_size=1)

    # Send linear and angular velocities
    test_msg = Twist()
    test_msg.linear.x = 1
    test_msg.angular.x = 0
    test_publisher.publish(test_msg)

    rospy.spin()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        test_publisher.publish(test_msg)
        rate.sleep()
