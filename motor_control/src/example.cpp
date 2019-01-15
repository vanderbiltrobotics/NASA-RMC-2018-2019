
// Include pheonix libraries
#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include "Platform-linux-socket-can.h"

// Include ROS libraries
#include "ros/ros.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

//void initDrive()
//{
//
//	// Set motors on one side of the robot to inverted mode
//	drive_talon_1.SetInverted(true);
//	drive_talon_2.SetInverted(true);
//
//}

int main(int argc, char **argv) {

    // Create a talon object for each motor controller

    // Initialize as a ROS node
    ros::init(argc, argv, "motor_control");
    ros::NodeHandle n;

    // Initialize the CAN inteface
	std::string interface;
	interface = "can0";
	ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

	VictorSPX v1(1);
//    TalonSRX drive_talon_1(0);
//    TalonSRX drive_talon_2(1);
//    TalonSRX drive_talon_3(2);
//    TalonSRX drive_talon_4(3);

	/* setup drive */
	//initDrive();

	return 0;
}
