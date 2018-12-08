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

const uint64_t MOVE = 0;
const uint64_t ROTATE = 1;

VictorSPX* fl = nullptr;
VictorSPX* fr = nullptr;
VictorSPX* bl = nullptr;
VictorSPX* br = nullptr;

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ctre::phoenix::platform::FeedWatchDog(100);
  double moveValue = msg->linear.x;
  double rotateValue = msg->angular.z;
  double leftMotorOutput = 0.0;
  double rightMotorOutput = 0.0;
  if (moveValue > 0.0) {
    if (rotateValue > 0.0) {
      leftMotorOutput = moveValue - rotateValue;
      rightMotorOutput = std::max(moveValue, rotateValue);
    } else {
      leftMotorOutput = std::max(moveValue, -rotateValue);
      rightMotorOutput = moveValue + rotateValue;
    }
  } else {
    if (rotateValue > 0.0) {
      leftMotorOutput = -std::max(-moveValue, rotateValue);
      rightMotorOutput = moveValue + rotateValue;
    } else {
      leftMotorOutput = moveValue - rotateValue;
      rightMotorOutput = -std::max(-moveValue, -rotateValue);
    }
  }
  fl->Set(ControlMode::PercentOutput, leftMotorOutput);
  fr->Set(ControlMode::PercentOutput, leftMotorOutput);
  bl->Set(ControlMode::PercentOutput, rightMotorOutput);
  br->Set(ControlMode::PercentOutput, rightMotorOutput);
  ROS_INFO("Move=%f Rotate=%f", moveValue, rotateValue);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("cmd_vel", 1000, cmdCallback);

    std::string interface = "can0";
    ctre::phoenix::platform::can::SetCANInterface(interface.c_str());

    fl = new VictorSPX(1);
    fr = new VictorSPX(2);
    bl = new VictorSPX(3);
    br = new VictorSPX(4);

    ros::spin();
    return 0;
}
