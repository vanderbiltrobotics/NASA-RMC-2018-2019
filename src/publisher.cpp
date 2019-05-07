#include "ros/ros.h"
#include "./navXTimeSync/AHRS.h"
#include "sensor_msgs/Imu.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navx");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::String>("gyro", 1000);
    ros::Rate loop_rate(10);
   
    int count(0);
    while (ros::ok())
    {
	sensor_msgs::Imu msg;	
	
	AHRS com = AHRS("/dev/ttyACM0");

	msg.header.seq = 0;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "0";

	msg.angular_velocity.x = com.GetRawGyroX(); 
	msg.angular_velocity.y = com.GetRawGyroY();
	msg.angular_velocity.z = com.GetRawGyroZ();
	msg.angular_velocity_covariance = {-1, 0, 0, 
					    0, 0, 0, 
					    0, 0, 0}; 

	msg.linear_acceleration.x = com.GetRawAccelX();
	msg.linear_acceleration.y = com.GetRawAccelY();
	msg.linear_acceleration.z = com.GetRawAccelZ();
	msg.linear_acceleration_covariance = {-1, 0, 0, 
					       0, 0, 0, 
					       0, 0, 0};

	msg.orientation.x = com.GetQuaternionX();
	msg.orientation.y = com.GetQuaternionY();
	msg.orientation.z = com.GetQuaternionZ();
	msg.orientation_covariance = {-1,  0,  0, 
				       0, -1,  0, 
				       0,  0, -1};

	ROS_INFO("Angular Velocity: 	(%s, %s, %s)\n
		  Linear Acceleration:  (%s, %s, %s)\n
		  Orientation: 		(%s, %s, %s)\n",
		  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
		  msg.linear_acceleration.x, msg.linear_velocity.y, msg.linear_velocity.z,
		  msg.orientation.x, msg.orientation.y, msg.orientation.z);

	pub.publish(msg);

	ros.spinOnce();

	loop_rate.sleep();
	++count;
    }

    return 0;
}
