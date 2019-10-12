#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64

class SpeedHandler:

    def __init__(self):

        # Drive motors
        self.drive_bl = 0.0
        self.drive_br = 0.0
        self.drive_fl = 0.0
        self.drive_fr = 0.0

        # Dig motors
        self.dig_hng = 0.0
        self.dig_lsc = 0.0
        self.dig_bkt = 0.0
        self.dig_cnv = 0.0

    def update_bl(self, msg):
        self.drive_bl = msg.data

    def update_br(self, msg):
        self.drive_br = msg.data

    def update_fl(self, msg):
        self.drive_fl = msg.data

    def update_fr(self, msg):
        self.drive_fr = msg.data

    def update_hng(self, msg):
        self.dig_hng = msg.data

    def update_lsc(self, msg):
        self.dig_lsc = msg.data

    def update_bkt(self, msg):
        self.dig_bkt = msg.data

    def update_cnv(self, msg):
        self.dig_cnv = msg.data

if __name__ == "__main__":

    # Initialize ROS node
    rospy.init_node("motor_speed_summary_publisher")

    # Create speed handler
    handler = SpeedHandler()

    # Attach subscribers
    rospy.Subscriber("back_left/set_percent_output", Float64, handler.update_bl)
    rospy.Subscriber("back_right/set_percent_output", Float64, handler.update_br)
    rospy.Subscriber("front_left/set_percent_output", Float64, handler.update_fl)
    rospy.Subscriber("front_right/set_percent_output", Float64, handler.update_fr)
    rospy.Subscriber("hinge_motor/set_percent_output", Float64, handler.update_hng)
    rospy.Subscriber("lead_screw_motor/set_percent_output", Float64, handler.update_lsc)
    rospy.Subscriber("bucket_motor/set_percent_output", Float64, handler.update_bkt)
    rospy.Subscriber("conveyor_motor/set_percent_output", Float64, handler.update_cnv)

    # Create publisher
    text_pub = rospy.Publisher("motor_speed_summary", String, queue_size=0)

    # Periodically publish summary message
    loop_rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # Create and publish message
        text = "BR: {0:.2f}  BL: {1:.2f}  FR: {2:.2f}  FL: {3:.2f}".format(
            handler.drive_br,
            handler.drive_bl,
            handler.drive_fr,
            handler.drive_fl
        )

        text += "    ||    HNG: {0:.2f}  LSC: {1:.2f}  BKT: {2:.2f}  CNV: {3:.2f}".format(
            handler.dig_hng,
            handler.dig_lsc,
            handler.dig_bkt,
            handler.dig_cnv
        )

        text_msg = String()
        text_msg.data = text
        text_pub.publish(text_msg)

        # Sleep at loop rate
        loop_rate.sleep()
