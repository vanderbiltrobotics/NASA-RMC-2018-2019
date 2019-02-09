#include "ros/ros.h"
#include "std_msgs/String.h"
#include "./navXTimeSync/AHRS.h"
#include "sensor_msgs/Imu.h"
#include <string>

using std::to_string;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navx");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::String>("gyro", 1000);
    ros::Rate loop_rate(10);
   
    int count(0);
    while (ros::ok())
    {
        std_msgs::String msg;
	
	AHRS com = AHRS("/dev/ttyACM0");

	msgs.data = to_string(com.GetPitch()) + to_string(com.GetRoll()) + to_string(com.GetYaw());

	ROS_INFO("%s", msg.data.c_str());

	pub.publish(msg);

	ros.spinOnce();

	loop_rate.sleep();
	++count;
    }

    return 0;
}
