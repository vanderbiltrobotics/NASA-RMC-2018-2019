#!/usr/bin/env python

# ----------------------------- #
# DESCRIPTION OF THIS NODE HERE #
# ----------------------------- #

# import required packages
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float64, Float32
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt


# Run the node
if __name__ == '__main__':

    # Initialize as ROS node
    rospy.init_node('test_webpage')

    # Initialize publishers
    elapsed_time_pub = rospy.Publisher("updates/elapsed_time", Float64, queue_size=0)
    battery_level_pub = rospy.Publisher("updates/battery_level", Float32, queue_size=0)
    robot_pose_pub = rospy.Publisher("updates/robot_pose", Pose, queue_size=0)

    # # Messages to publish # #

    # Elapsed time (Float64)
    et_msg = Float64()
    et_msg.data = 0.0

    # Battery level (Float32)
    bl_msg = Float32()
    bl_msg.data = 12.2

    # Robot pose (Pose)
    rp_msg = Pose()
    rp_msg.position.x = 2.0
    rp_msg.position.y = 1.0
    rp_msg.position.z = 0.0
    rp_msg.orientation.x = 0
    rp_msg.orientation.y = 0
    rp_msg.orientation.z = 0
    rp_msg.orientation.w = 1

    rate = rospy.Rate(50)
    print "here-------------------------------------------------------"
    while not rospy.is_shutdown():

        # et_msg.data = et_msg.data + 1
        # elapsed_time_pub.publish(et_msg)
        # battery_level_pub.publish(bl_msg)
        robot_pose_pub.publish(rp_msg)
        rate.sleep()