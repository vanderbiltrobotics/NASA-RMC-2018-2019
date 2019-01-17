
// General C++ includes
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstdint>

// Phoenix libraries
#include "ctre/Phoenix.h"
#include "Platform-linux-socket-can.h"

// ROS libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Global pointers for each motor controller

int main(int argc, char **argv) {

    // Initialize ROS node
    ros::init(argc, argv, "can_bus_interface");
    ros::NodeHandle n;

    // Initialize CAN bus
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    // Create a Talon object
    ctre::phoenix::motorcontrol::can::TalonSRX fl(1);

    ros::Rate loop_rate(1);

    // Loop
    while (ros::ok())
    {
        // Read information from the Talon
        std::cout << "Current voltage: " << fl.GetBusVoltage() << std::endl;

        // Pause at loop rate
        loop_rate.sleep();
    }


    return 0;
}
