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

	msg.header.stamp = ros::Time::now();

	msg.angular_velocity.x = AHRSPosUpdate.vel_x;
	msg.angular_velocity.y = AHRSPosUpdate.vel_y;
	msg.angular_velocity.z = AHRSPosUpdate.vel_z;
	msg.angular_velocity_covariance = {-1, 0, 0, 0, 0, 0, 0, 0, 0};

	msg.linear_acceleration.x = AHRSUpdateBase.linear_accel_x;
	msg.linear_acceleration.y = AHRSUpdateBase.linear_accel_y;
	msg.linear_acceleration.z = AHRSUpdateBase.linear_accel_z;
	msg.linear_acceleration_covariance = {-1, 0, 0, 0, 0, 0, 0, 0, 0};

	msg.orientation.x = AHRSUpdateBase.quat_x;
	msg.orientation.y = AHRSUpdateBase.quat_y;
	msg.orientation.z = AHRSUpdateBase.quat_z
	msg.orientation_covariance = {-1, 0, 0, 0, 0, 0, 0, 0, 0};

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
