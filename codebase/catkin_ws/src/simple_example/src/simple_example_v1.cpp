

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

    // Initializes a node called "talker"
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    // Create a publisher that sends String messages on a topic called "JakesComputer"
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("JakesComputer", 1000);

    // Defines a loop rate of 10
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {

        // Create new message
        std_msgs::String msg;

        // String to use as message
        std::string text = "hello world:   " + std::to_string(count);

        // Fill our ROS message with the text
        msg.data = text;

        // Publish the message
        chatter_pub.publish(msg);

        // Spinning tells ROS to publish all un-published messages
        ros::spinOnce();

        // Sleep's the appropriate time so that this loop runs 'loop_rate' times per second
        loop_rate.sleep();

        // Increment count
        ++count;
    }


    return 0;
}