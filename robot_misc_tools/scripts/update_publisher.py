#!/usr/bin/env python

# ----------------------------- #
# Takes mission control information, updates, and publishes it at a slower rate #
# ----------------------------- #

# imports
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *

class UpdatePublisher:

    def __init__ (self, topic_name, message_type):

        #Initialize variable to be updated
        self.latest_var = 0

        #Initialize Publisher
        self.publisher = rospy.Publisher('updates/' + topic_name, message_type, queue_size=0)

        #Initialize Subscriber
        self.subscriber = rospy.Subscriber(topic_name, message_type, self.update_message)

    # Update the variable when a message is published on the main topic
    def update_message(self, data):
        self.latest_var = data

    # Publish an update
    def publish_message(self):
        self.publisher.publish(self.latest_var)

    @staticmethod
    def load_config_file(file_name):

        updaters = []

        # Read the config file passed by file name
        config_file = open(file_name)
        updaters_list = config_file.readlines()
        updaters_list = [item.replace("\n", "") for item in updaters_list]
        config_file.close()

        # Loop and append the updaters list with UpdatePublishers
        for item in updaters_list:
            comma = item.find(",")
            topic_name = item[:comma]
            message_type = item[comma+2:]
            updaters.append(UpdatePublisher(topic_name, message_type))

        return updaters

if __name__ == "__main__":

    #initialize ros node
    rospy.init_node('update_node')

    #Read a file containing a bunch of [topic name, message type] pairs, create an UpdatePublisher for each
    updaters = UpdatePublisher.load_config_file("update_configs/test_config.csv")

    #read values from server
    update_rate = rospy.get_param("update_rate")

    #Log initialization message
    rospy.loginfo("Update node initialized...")

    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():

        # Send an update from each UpdatePublisher in updaters list
        for item in updaters:
            item.update_message()
            item.publish_message()

        rate.sleep()
