#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("heard [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navx");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("gyro", 1000, callback);
   
    ros::spin();

    return 0;
}
