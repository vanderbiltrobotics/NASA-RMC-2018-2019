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

    def __init__ (self, topic_name):

        #Initialize variable to be updated
        latest_var = 0

        #Initialize Publisher
        self.publisher = rospy.Publisher('updates/' + topic_name, Float64, queue_size=0)

        #Initialize Subscriber
        self.subscriber = rospy.Subscriber(topic_name, Float64, self.update_message)

    def update_message(self, data):

        #Update the variable
        latest_var = data
        return latest_var


    def publish_message(self, latest_var):

        #Publish the variable
        self.publisher.publish(latest_var)

    @staticmethod
    def load_config_file(file_name):

        updaters = []

        # Create a list of UpdatePublishers

        return updaters

if __name__ == "__main__":

    #initialize ros node
    rospy.init_node('update_node')

    #create UpdatePublisher object
    updater = UpdatePublisher()

    # TODO: Read a file containing a bunch of [topic name, message type] pairs, create an UpdatePublisher for each
    # updaters = load_config_file(filename)

    #read values from server
    update_rate = rospy.get_param("update_rate")

    #Log initialization message
    rospy.loginfo("Update node initialized...")

    rate = rospy.Rate(update_rate)
    while not rospy.is_shutdown():

        # Send an update from each UpdatePublisher

        rate.sleep()
