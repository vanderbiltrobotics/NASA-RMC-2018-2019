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
    test_msg.linear.x = 0
    test_msg.angular.z = 1

    while test_publisher.get_num_connections() < 1:
        rospy.sleep(1)

    test_publisher.publish(test_msg)
