#include "ctre/Phoenix.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstdint>
#include "Platform-linux-socket-can.h"

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

int main(int argc, char **argv) {

    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    std::cout << "Reached this point" << std::endl;

    return 0;
}
