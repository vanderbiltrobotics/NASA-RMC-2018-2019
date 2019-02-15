#!/usr/bin/env python

# ----------------------------- #
# Takes mission control information, updates, and publishes it at a slower rate #
# ----------------------------- #

# imports
import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, Float32

class UpdatePublisher:

    def __init__ (self, topic_name):

        #Initialize variable to be updated
        latest_var = 0

        #Initialize Publisher
        self.publisher = rospy.publisher('updates/' + topic_name, Float64, queue_size=0)

        #Initialize Subscriber
        self.subscriber = rospy.subscriber(topic_name, Float64, self.update_message)

    def update_message(self, data):

        #Update the variable
        latest_var = data
        return latest_var


    def publish_message(self, latest_var):

        #Publish the variable
        self.publisher.publish(latest_var)


if __name__ == "__main__":

    #initialize ros node
    rospy.init_node('update_node')

    #create UpdatePublisher object
    updater = UpdatePublisher()

    #read values from server
    update_rate = rospy.get_param("update_rate")

    #Log initialization message
    rospy.loginfo("Update node initialized...")

    rate = rospy.rate(update_rate)
    while not rospy.is_shutdown():
        rospy.sleep()
