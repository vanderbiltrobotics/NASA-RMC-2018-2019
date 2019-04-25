#!/usr/bin/env python

# ----------------------------- #
# Collects data from other topics and stores information using rosbag#
# ----------------------------- #

# imports
import rospy
import rosbag
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *

class dataCollector:

    def __init__(self, topic_name, message_type):
        # initial creation of a bag of data
        self.data_bag = rospy.Bag('data.bag', 'w')

        # initialize subscsribers
        self.subscriber = rospy.Subscriber(topic_name, message_type, self.callback)

    #some more methods using bag
    def callback(self, data, topic_name):

        # put the data into the bag
        self.data_bag.write(self, topic_name, data)

if __name__ == "__main__":

    # initialize the node
    rospy.init_node('data_collector')

    while not rospy.is_shutdown():
        rospy.sleep()