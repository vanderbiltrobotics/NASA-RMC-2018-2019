
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

const double WHEEL_DIAMETER = 0.15;
const double ROBOT_WIDTH = 0.75; //measured from the centers of the wheels
// if that matter for anything


// getWheelAngularVelocity
// this method calculates the required angular velocity of a wheel for the robot to turn the required distance
//pre:
//   sign conventions
//   forward and counterclockwise from the center of the robot are the positive
//   senses for both the linear and the angular velocity invalid_argument
//   side: 1 for right side of center, -1 for left side
double getWheelAngularVelocity(double linV, double angV, int side) {
    if (side != -1 && side != 1) {
        throw std::invalid_argument("'side' parameter invalid");
    } else {
        double out = 0.5 * ROBOT_WIDTH * angV;
        out = linV + (side * out);
        out = out / (WHEEL_DIAMETER / 2);
        return out;
    }
}


int main(int argc, char **argv) {

    // Initialize ROS node
    ros::init(argc, argv, "can_bus_interface");
    ros::NodeHandle n;

    // Initialize CAN bus
    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    // Create a Talon object
    ctre::phoenix::motorcontrol::can::TalonSRX fl(1);

    // Wait for CAN bus set up to complete
    ros::Duration setup_time(2);
    setup_time.sleep();

    // Drive wheel
    fl.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 20);

    // Rate to loop at (in Hz)
    ros::Rate loop_rate(1);

    // Loop
    while (ros::ok()) {
        // Read information from the Talon
        std::cout << "Current voltage: " << fl.GetBusVoltage() << std::endl;

        // Pause at loop rate
        loop_rate.sleep();
    }


    return 0;
}
