#include "ctre/Phoenix.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <cstdint>
#include <mutex>
#include "Platform-linux-socket-can.h"

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "can_bus_interface/TalonConfig.h"


TalonSRX* talon = nullptr;

void reconfigure(motor_control::TalonConfig &config, uint32_t level) {

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talon_srx");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<motor_control::TalonConfig> server;
    dynamic_reconfigure::Server<motor_control::TalonConfig>::CallbackType f;

    f = boost::bind(&reconfigure, _1, _2);
    server.setCallback(f);

    std::string interface;
    nh.param<std::string>("interface", interface, "can0");
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    int id;
    if(!nh.getParam("id", id)){
        throw std::invalid_argument("Missing Device ID parameter!");
    }
    talon = new TalonSRX(id);


    motor_control::TalonConfig config;
    config.interface = nh.getParam("interface", config.interface);
    config.id = nh.getParam("id", config.id);

    reconfigure(config, 0);

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
